include("$(PORT_DIR)/boards/manifest.py")

require("bundle-networking")

# SD Card
require("sdcard")

# Bluetooth
require("aioble")

# Picographics
require("st7789")

include("../manifest_picow.py")
