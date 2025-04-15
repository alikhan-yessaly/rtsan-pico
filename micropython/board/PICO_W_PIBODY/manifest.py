include("$(PORT_DIR)/boards/manifest.py")

require("bundle-networking")
require("umqtt.simple")

# SD Card
require("sdcard")

# Bluetooth
require("aioble")

# include("../manifest_picow.py")
freeze("../../modules_py", "boot.py")
