# Oh My Pendulum - VCV Rack Plugin
# Professional Plugin Development Structure

RACK_DIR ?= ../Rack-SDK

# Add sources
SOURCES += $(wildcard src/*.cpp)

# Add resources to distribution
DISTRIBUTABLES += res
DISTRIBUTABLES += LICENSE

include $(RACK_DIR)/plugin.mk

# Rack-free unit tests
TEST_SRC := $(wildcard tests/*.cpp)
TEST_FLAGS ?=

.PHONY: test
test:
	@mkdir -p build
	$(CXX) -std=c++11 -O2 -Wall -Wextra $(TEST_FLAGS) -Isrc $(TEST_SRC) -o build/test_ohmymodular
	./build/test_ohmymodular
