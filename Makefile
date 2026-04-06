CXX ?= c++
APP_NAME ?= Stratfinder
TARGET ?= $(APP_NAME)
VERSION ?= dev
ENABLE_LTO ?= 0
UNAME_S := $(shell uname -s)
ARCH ?= $(shell uname -m)
BUILD_DIR := build/$(UNAME_S)-$(ARCH)
PKG_CONFIG ?= pkg-config

SRC := \
	src/main.cpp \
	src/zEngine.cpp \
	src/zSolver.cpp \
	src/player.cpp \
	src/inputCracker.cpp \
	src/segLerp.cpp \
	src/gui/InputCrackerTab.cpp \
	src/gui/ZSolverTab.cpp \
	src/gui/JumpFinderTab.cpp \
	third_party/imgui/imgui.cpp \
	third_party/imgui/imgui_draw.cpp \
	third_party/imgui/imgui_widgets.cpp \
	third_party/imgui/imgui_tables.cpp \
	third_party/imgui/backends/imgui_impl_glfw.cpp \
	third_party/imgui/backends/imgui_impl_opengl3.cpp \
	third_party/imgui/misc/cpp/imgui_stdlib.cpp

OBJ := $(addprefix $(BUILD_DIR)/,$(SRC:.cpp=.o))
DEP := $(OBJ:.o=.d)

CPPFLAGS := -Isrc -Ithird_party/imgui -Ithird_party/imgui/backends
CXXFLAGS := -std=c++20 -Wall -Wextra -MMD -MP -ffp-contract=off
LDFLAGS :=
LDLIBS := -lglfw
RELEASE_CXXFLAGS :=
RELEASE_LDFLAGS :=
STRIP_CMD :=

ifeq ($(UNAME_S),Darwin)
CXX := clang++
ifeq ($(ARCH),x86_64)
BREW_PREFIX ?= /usr/local
else
BREW_PREFIX ?= /opt/homebrew
endif
CPPFLAGS += -I$(BREW_PREFIX)/include -DGL_SILENCE_DEPRECATION
LDFLAGS += -L$(BREW_PREFIX)/lib \
	-framework OpenGL \
	-framework Cocoa \
	-framework IOKit \
	-framework CoreVideo
RELEASE_LDFLAGS += -Wl,-dead_strip
STRIP_CMD = strip -x $(BUILD_DIR)/$(TARGET)
endif

ifeq ($(UNAME_S),Linux)
CPPFLAGS += $(shell $(PKG_CONFIG) --cflags glfw3 2>/dev/null)
LDLIBS += $(shell $(PKG_CONFIG) --libs glfw3 2>/dev/null) -lGL -ldl -lpthread
LDLIBS := $(filter-out -lglfw,$(LDLIBS))
RELEASE_LDFLAGS += -Wl,--gc-sections
STRIP_CMD = strip --strip-unneeded $(BUILD_DIR)/$(TARGET)
endif

ifneq (,$(findstring MINGW,$(UNAME_S)))
LDLIBS := -lglfw3 -lopengl32 -lgdi32 -lshell32
RELEASE_LDFLAGS += -Wl,--gc-sections
STRIP_CMD = strip --strip-unneeded $(BUILD_DIR)/$(TARGET)
endif
ifneq (,$(findstring MSYS,$(UNAME_S)))
LDLIBS := -lglfw3 -lopengl32 -lgdi32 -lshell32
RELEASE_LDFLAGS += -Wl,--gc-sections
STRIP_CMD = strip --strip-unneeded $(BUILD_DIR)/$(TARGET)
endif
ifeq ($(OS),Windows_NT)
LDLIBS := -lglfw3 -lopengl32 -lgdi32 -lshell32
RELEASE_LDFLAGS += -Wl,--gc-sections
STRIP_CMD = strip --strip-unneeded $(BUILD_DIR)/$(TARGET)
endif

ifeq ($(ENABLE_LTO),1)
RELEASE_CXXFLAGS += -flto
RELEASE_LDFLAGS += -flto
endif

.PHONY: all debug release clean clean-artifacts \
	package-macos-arm64 package-macos-x86_64 package-linux-x86_64 package-windows-x86_64

all: debug

debug: CXXFLAGS += -O0 -g
debug: $(BUILD_DIR)/$(TARGET)

release: CXXFLAGS += -O3 -DNDEBUG -fdata-sections -ffunction-sections $(RELEASE_CXXFLAGS)
release: LDFLAGS += $(RELEASE_LDFLAGS)
release: $(BUILD_DIR)/$(TARGET)
ifneq ($(STRIP_CMD),)
	$(STRIP_CMD)
endif

$(BUILD_DIR)/$(TARGET): $(OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(OBJ) $(LDFLAGS) $(LDLIBS) -o $@
	@mkdir -p build
	cp -f $@ build/$(TARGET)

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(BUILD_DIR)/$(TARGET) $(OBJ) $(DEP)

clean-artifacts:
	rm -rf build/*.dSYM dist

package-macos-arm64:
	@host="$$(uname -s)"; \
	if [ "$$host" != "Darwin" ]; then \
		echo "package-macos-arm64 must be run on macOS."; \
		exit 2; \
	fi
	$(MAKE) clean release UNAME_S=Darwin ARCH=arm64 TARGET=$(APP_NAME)
	VERSION=$(VERSION) APP_NAME=$(APP_NAME) ./scripts/package.sh macos arm64 build/Darwin-arm64/$(APP_NAME)

package-macos-x86_64:
	@host="$$(uname -s)"; \
	if [ "$$host" != "Darwin" ]; then \
		echo "package-macos-x86_64 must be run on macOS."; \
		exit 2; \
	fi
	$(MAKE) clean release UNAME_S=Darwin ARCH=x86_64 TARGET=$(APP_NAME)
	VERSION=$(VERSION) APP_NAME=$(APP_NAME) ./scripts/package.sh macos x86_64 build/Darwin-x86_64/$(APP_NAME)

package-linux-x86_64:
	@host="$$(uname -s)"; \
	if [ "$$host" != "Linux" ]; then \
		echo "package-linux-x86_64 must be run on Linux."; \
		exit 2; \
	fi
	$(MAKE) clean release UNAME_S=Linux ARCH=x86_64 TARGET=$(APP_NAME)
	VERSION=$(VERSION) APP_NAME=$(APP_NAME) ./scripts/package.sh linux x86_64 build/Linux-x86_64/$(APP_NAME)

package-windows-x86_64:
	@host="$$(uname -s)"; \
	case "$$host" in MINGW*|MSYS*|CYGWIN*) ;; \
	*) echo "package-windows-x86_64 must be run in MinGW/MSYS/Cygwin."; exit 2;; \
	esac
	$(MAKE) clean release UNAME_S=MINGW64_NT ARCH=x86_64 TARGET=$(APP_NAME).exe
	VERSION=$(VERSION) APP_NAME=$(APP_NAME) ./scripts/package.sh windows x86_64 build/MINGW64_NT-x86_64/$(APP_NAME).exe

-include $(DEP)
