# Oh My Pendulum - VCV Rack Plugin
# Professional Plugin Development Structure

RACK_DIR ?= ../Rack-SDK

# Add sources
SOURCES += $(wildcard src/*.cpp)

# Add resources to distribution
DISTRIBUTABLES += res
DISTRIBUTABLES += LICENSE

include $(RACK_DIR)/plugin.mk

# Rack-free unit tests (no SDK or plugin build required).
.PHONY: test
test:
	@mkdir -p build
	$(CXX) -std=c++11 -O2 -Wall -Wextra -Isrc tests/test_expander_math.cpp -o build/test_expander_math
	./build/test_expander_math
