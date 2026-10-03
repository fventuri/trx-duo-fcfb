package fcfb

import (
	"fmt"
	"net"
	"strconv"
)

// checkHostMTU verifies that the local Ethernet interface used to reach the board
// is configured for the MTU declared in the config. The board streams jumbo UDP
// datagrams (MTU 3980); if the host interface is left at 1500 the jumbo frames are
// silently dropped by the host kernel and the farm decodes nothing -- a failure
// mode that gives no error, just zero spots. This turns that into a fast, loud
// failure at startup.
//
// wantMTU <= 0 means "not configured" -> no check (the config default). Otherwise
// the host interface MTU must equal wantMTU exactly.
func checkHostMTU(boardIP string, udpPort, wantMTU int) error {
	if wantMTU <= 0 {
		return nil
	}
	iface, haveMTU, err := hostIfaceForPeer(boardIP, udpPort)
	if err != nil {
		return fmt.Errorf("mtu check: %w", err)
	}
	if haveMTU != wantMTU {
		return fmt.Errorf("mtu mismatch: config mtu=%d but host interface %q reaching the board is mtu=%d; "+
			"the board streams jumbo frames and a smaller host MTU is silently dropped -- run: sudo ip link set %s mtu %d",
			wantMTU, iface, haveMTU, iface, wantMTU)
	}
	return nil
}

// hostIfaceForPeer returns the name and MTU of the local interface the kernel
// would use to reach boardIP. It opens a connected UDP socket (which selects the
// route and local address without sending any packet) and then matches the chosen
// local IP against the host's interface addresses.
func hostIfaceForPeer(boardIP string, udpPort int) (name string, mtu int, err error) {
	c, err := net.Dial("udp", net.JoinHostPort(boardIP, strconv.Itoa(udpPort)))
	if err != nil {
		return "", 0, fmt.Errorf("cannot determine route to board %s: %w", boardIP, err)
	}
	defer c.Close()
	local, ok := c.LocalAddr().(*net.UDPAddr)
	if !ok {
		return "", 0, fmt.Errorf("unexpected local address type %T", c.LocalAddr())
	}

	ifaces, err := net.Interfaces()
	if err != nil {
		return "", 0, err
	}
	for _, ifc := range ifaces {
		addrs, err := ifc.Addrs()
		if err != nil {
			continue
		}
		for _, a := range addrs {
			var ip net.IP
			switch v := a.(type) {
			case *net.IPNet:
				ip = v.IP
			case *net.IPAddr:
				ip = v.IP
			}
			if ip != nil && ip.Equal(local.IP) {
				return ifc.Name, ifc.MTU, nil
			}
		}
	}
	return "", 0, fmt.Errorf("no host interface owns local address %s (route to board %s)", local.IP, boardIP)
}
