#!/usr/bin/env bash
# Wi-Fi-agnostic robot discovery.
#
# Probes ssh rosbot. If unreachable, tries husarion.local (mDNS), then a
# full /24 ping sweep + arp grep. On success, rewrites the HostName line of
# the `Host rosbot` block in ~/.ssh/config so every other tool just works.
#
# Usage:
#   scripts/find_robot.sh                # discover; rewrite ssh config
#   scripts/find_robot.sh --quiet        # same, only print final IP on stdout
#   scripts/find_robot.sh --probe-only   # exit 0 if reachable, 1 if not, no rewrite
#
# Exit codes: 0 reachable, 1 not found.
set -euo pipefail

ROBOT="${ROBOT:-rosbot}"
QUIET=0
PROBE_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --quiet) QUIET=1 ;;
    --probe-only) PROBE_ONLY=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
  esac
done

info() { (( QUIET )) || printf "[find] %s\n" "$*" >&2; }
warn() { printf "[find] WARN: %s\n" "$*" >&2; }

probe_ssh() {
  ssh -o ConnectTimeout=4 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$1" 'true' >/dev/null 2>&1
}

current_hostname() {
  # Resolve what `Host rosbot` currently points to in ~/.ssh/config.
  awk '
    /^Host / { in_block = ($2 == "rosbot") }
    in_block && /^[[:space:]]*HostName[[:space:]]/ { print $2; exit }
  ' "$HOME/.ssh/config"
}

current_identity() {
  # Resolve which key the rosbot host uses, with ~ expansion. Falls back to
 # ~/.ssh/id_0xanb (documented key in ) if unset.
  local id
  id=$(awk '
    /^Host / { in_block = ($2 == "rosbot") }
    in_block && /^[[:space:]]*IdentityFile[[:space:]]/ { print $2; exit }
  ' "$HOME/.ssh/config")
  id="${id/#\~/$HOME}"
  [[ -z "$id" || ! -f "$id" ]] && id="$HOME/.ssh/id_0xanb"
  printf '%s\n' "$id"
}

mdns_lookup() {
  # husarion.local — works on most home Wi-Fi and direct-tether, blocked on
  # some hotel / Xiaomi routers.
  local ip
  ip=$(dscacheutil -q host -a name husarion.local 2>/dev/null \
        | awk '/ip_address:/{print $2; exit}')
  [[ -n "$ip" ]] && printf '%s\n' "$ip"
}

arp_sweep() {
  # /24 ping flood, then grep arp table for any host advertising husarion / rosbot.
  local iface my_ip prefix i
  iface=$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')
  [[ -z "$iface" ]] && return 1
  my_ip=$(ifconfig "$iface" 2>/dev/null | awk '/inet [0-9]/{print $2; exit}')
  [[ -z "$my_ip" ]] && return 1
  prefix="${my_ip%.*}"
  info "scanning ${prefix}.0/24 for husarion (Mac is at ${my_ip} on ${iface})"
  for i in $(seq 1 254); do
    ping -c 1 -W 100 -q "${prefix}.${i}" >/dev/null 2>&1 &
    (( i % 50 == 0 )) && wait
  done
  wait
  arp -a 2>/dev/null \
    | grep -iE 'husarion|rosbot' \
    | grep -oE '\(([0-9]+\.){3}[0-9]+\)' \
    | tr -d '()' \
    | head -1
}

tcp_ssh_sweep() {
  # ICMP-blind discovery for networks where ARP-grep fails: iPhone Personal
  # Hotspot, eduroam (client-isolation enabled), most conference Wi-Fi.
  # Probes TCP/22 in parallel across the local /24, then SSH-key-tests every
  # responder against husarion@ to disambiguate the robot from random other
  # hosts that happen to have port 22 open. Returns the first authenticating
  # IP.
  local iface my_ip prefix i identity ip results candidates=()
  iface=$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')
  [[ -z "$iface" ]] && return 1
  my_ip=$(ifconfig "$iface" 2>/dev/null | awk '/inet [0-9]/{print $2; exit}')
  [[ -z "$my_ip" ]] && return 1
  prefix="${my_ip%.*}"
  identity=$(current_identity)
  info "TCP/22 sweeping ${prefix}.0/24 for any SSH responder (ICMP-blind path)"
  results=$(
    for i in $(seq 1 254); do
      ip="${prefix}.${i}"
      [[ "$ip" == "$my_ip" ]] && continue
      (nc -z -G 1 -w 1 "$ip" 22 2>/dev/null && echo "$ip") &
      (( i % 50 == 0 )) && wait
    done
    wait
  )
  while IFS= read -r ip; do
    [[ -n "$ip" ]] && candidates+=("$ip")
  done <<< "$results"
  [[ ${#candidates[@]} -eq 0 ]] && return 1
  info "  responders: ${candidates[*]}"
  for ip in "${candidates[@]}"; do
    if ssh -o ConnectTimeout=3 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
           -o UserKnownHostsFile="$HOME/.ssh/known_hosts" \
           -i "$identity" "husarion@${ip}" 'true' >/dev/null 2>&1; then
      printf '%s\n' "$ip"
      return 0
    fi
  done
  return 1
}

update_ssh_hostname() {
  local new_ip="$1"
  local sshcfg="$HOME/.ssh/config"
  [[ -f "$sshcfg" ]] || { warn "ssh config $sshcfg missing"; return 1; }
  local cur
  cur=$(current_hostname)
  if [[ "$cur" == "$new_ip" ]]; then
    info "ssh config already points Host ${ROBOT} → ${new_ip}"
    return 0
  fi
  cp "$sshcfg" "$HOME/.ssh/config.par-a3.bak"
  awk -v ip="$new_ip" '
    BEGIN { in_block=0 }
    /^Host rosbot$/      { in_block=1; print; next }
    in_block && /^Host / { in_block=0 }
    in_block && /^[[:space:]]*HostName[[:space:]]/ { print "    HostName " ip; next }
    { print }
  ' "$sshcfg" > "$sshcfg.new" && mv "$sshcfg.new" "$sshcfg"
  info "ssh config: Host ${ROBOT} HostName ${cur:-?} → ${new_ip}"
  # Drop any stale host key for the new IP so the next ssh accepts cleanly.
  ssh-keygen -R "$new_ip" >/dev/null 2>&1 || true
}

# 1. Cheap probe — current alias.
if probe_ssh "$ROBOT"; then
  info "ssh ${ROBOT} reachable (current config: $(current_hostname))"
  (( QUIET )) && printf '%s\n' "$(current_hostname)"
  exit 0
fi
(( PROBE_ONLY )) && exit 1

# 2. mDNS lookup (no /24 sweep needed if this answers).
mdns_ip=$(mdns_lookup || true)
if [[ -n "${mdns_ip:-}" ]] && probe_ssh "husarion.local"; then
  info "found via mDNS at ${mdns_ip}"
  update_ssh_hostname "${mdns_ip}"
  (( QUIET )) && printf '%s\n' "${mdns_ip}"
  exit 0
fi

# 3. Full /24 sweep + arp grep.
swept_ip=$(arp_sweep || true)
if [[ -n "${swept_ip:-}" ]]; then
  info "found via arp sweep at ${swept_ip}"
  update_ssh_hostname "${swept_ip}"
  if probe_ssh "$ROBOT"; then
    (( QUIET )) && printf '%s\n' "${swept_ip}"
    exit 0
  fi
  warn "rewrote ssh config to ${swept_ip} but ssh still fails — check User / IdentityFile in Host ${ROBOT}"
  exit 1
fi

# 4. TCP/22 + SSH-key sweep — for ICMP-blocking networks (iPhone hotspot,
# eduroam with client-isolation, most conference Wi-Fi). Slower than ARP but
# survives every network we have seen.
tcp_ip=$(tcp_ssh_sweep || true)
if [[ -n "${tcp_ip:-}" ]]; then
  info "found via TCP/SSH sweep at ${tcp_ip}"
  update_ssh_hostname "${tcp_ip}"
  if probe_ssh "$ROBOT"; then
    (( QUIET )) && printf '%s\n' "${tcp_ip}"
    exit 0
  fi
  warn "rewrote ssh config to ${tcp_ip} but ssh alias still fails — check User / IdentityFile in Host ${ROBOT}"
  exit 1
fi

warn "robot not found on ${ROBOT}, mDNS, ARP sweep, or TCP/SSH sweep. Power-check + Wi-Fi-check the robot."
exit 1
