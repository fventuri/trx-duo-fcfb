# fcfb ships prebuilt target binaries (fcfb_server, fcfb_capture) in the rootfs
# overlay rather than as Buildroot packages, so there are no package .mk files to
# include here. The wildcard is kept for parity with the base pattern and is a
# harmless no-op while package/ is empty.
include $(sort $(wildcard $(BR2_EXTERNAL_TRXDUO_FCFB_PATH)/package/*/*.mk))
