# Oh My Pendulum - VCV Rack Plugin
# Professional Plugin Development Structure

RACK_DIR ?= ../Rack-SDK

# Add sources
SOURCES += $(wildcard src/*.cpp)

# Add resources to distribution
DISTRIBUTABLES += res
DISTRIBUTABLES += LICENSE

include $(RACK_DIR)/plugin.mk
