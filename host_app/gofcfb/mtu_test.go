package fcfb

import (
	"net"
	"strings"
	"testing"
)

// loopbackMTU returns the MTU the resolver would use for a loopback peer, or
// skips the test if loopback can't be resolved (unusual, but keeps CI honest).
func loopbackMTU(t *testing.T) int {
	t.Helper()
	name, mtu, err := hostIfaceForPeer("127.0.0.1", 55055)
	if err != nil {
		t.Skipf("cannot resolve loopback interface: %v", err)
	}
	if mtu <= 0 {
		t.Fatalf("loopback interface %q has non-positive MTU %d", name, mtu)
	}
	return mtu
}

func TestCheckHostMTU_Unset(t *testing.T) {
	// wantMTU <= 0 means "not configured" -> no check, no route lookup.
	if err := checkHostMTU("192.0.2.1", 55055, 0); err != nil {
		t.Fatalf("wantMTU=0 should skip the check, got %v", err)
	}
}

func TestCheckHostMTU_Match(t *testing.T) {
	mtu := loopbackMTU(t)
	if err := checkHostMTU("127.0.0.1", 55055, mtu); err != nil {
		t.Fatalf("matching MTU should pass, got %v", err)
	}
}

func TestCheckHostMTU_Mismatch(t *testing.T) {
	mtu := loopbackMTU(t)
	err := checkHostMTU("127.0.0.1", 55055, mtu+1)
	if err == nil {
		t.Fatal("mismatched MTU should fail")
	}
	if !strings.Contains(err.Error(), "mtu mismatch") {
		t.Fatalf("error should mention the mismatch, got %v", err)
	}
}

func TestHostIfaceForPeer_Loopback(t *testing.T) {
	name, mtu, err := hostIfaceForPeer("127.0.0.1", 55055)
	if err != nil {
		t.Skipf("cannot resolve loopback interface: %v", err)
	}
	// The chosen interface must actually own the loopback address.
	ifc, err := net.InterfaceByName(name)
	if err != nil {
		t.Fatalf("InterfaceByName(%q): %v", name, err)
	}
	if ifc.MTU != mtu {
		t.Fatalf("reported MTU %d != interface MTU %d", mtu, ifc.MTU)
	}
}
