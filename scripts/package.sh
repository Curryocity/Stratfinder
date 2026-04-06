#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <platform> <arch> <binary-path>" >&2
  exit 1
fi

platform="$1"
arch="$2"
binary_path="$3"
version="${VERSION:-dev}"
app_name="${APP_NAME:-Stratfinder}"
archive_output="${PACKAGE_ARCHIVE:-1}"
root_dir="$(cd "$(dirname "$0")/.." && pwd)"
asset_dir="$root_dir/asset"
dist_dir="$root_dir/dist"
mac_icon_icns="$asset_dir/icons/app.icns"
app_icon_png="$asset_dir/icons/app.png"

if [ ! -f "$binary_path" ]; then
  echo "Binary not found: $binary_path" >&2
  exit 1
fi
if [ ! -d "$asset_dir" ]; then
  echo "Asset directory not found: $asset_dir" >&2
  exit 1
fi

mkdir -p "$dist_dir"

is_system_windows_dll() {
  local dll_upper
  dll_upper="$(printf '%s' "$1" | tr '[:lower:]' '[:upper:]')"
  case "$dll_upper" in
    KERNEL32.DLL|KERNELBASE.DLL|NTDLL.DLL|USER32.DLL|GDI32.DLL|SHELL32.DLL|ADVAPI32.DLL|IMM32.DLL|OPENGL32.DLL|API-MS-WIN-*.DLL)
      return 0
      ;;
  esac
  return 1
}

copy_windows_dlls() {
  local exe_path="$1"
  local out_dir="$2"
  local -a search_dirs=()
  local -a queue=()
  local -a seen_bins=()
  local -a missing=()
  local -a path_dirs=()

  if ! command -v objdump >/dev/null 2>&1; then
    echo "objdump not found; cannot bundle DLL dependencies." >&2
    return 1
  fi

  if command -v g++ >/dev/null 2>&1; then
    search_dirs+=("$(dirname "$(command -v g++)")")
  fi
  IFS=':' read -r -a path_dirs <<< "${PATH:-}"
  for d in "${path_dirs[@]}"; do
    [ -n "$d" ] && search_dirs+=("$d")
  done
  search_dirs+=("$(dirname "$exe_path")")

  queue+=("$exe_path")
  local idx=0
  while [ "$idx" -lt "${#queue[@]}" ]; do
    local bin="${queue[$idx]}"
    idx=$((idx + 1))

    local seen=0
    local prior=""
    for prior in "${seen_bins[@]}"; do
      if [ "$prior" = "$bin" ]; then
        seen=1
        break
      fi
    done
    if [ "$seen" -eq 1 ]; then
      continue
    fi
    seen_bins+=("$bin")

    while IFS= read -r dll; do
      [ -z "$dll" ] && continue
      if is_system_windows_dll "$dll"; then
        continue
      fi

      local staged="$out_dir/$dll"
      if [ -f "$staged" ]; then
        queue+=("$staged")
        continue
      fi

      local found=""
      local d=""
      for d in "${search_dirs[@]}"; do
        if [ -f "$d/$dll" ]; then
          found="$d/$dll"
          break
        fi
      done

      if [ -n "$found" ]; then
        cp -f "$found" "$staged"
        queue+=("$staged")
      else
        local already_missing=0
        local item=""
        for item in "${missing[@]}"; do
          if [ "$item" = "$dll" ]; then
            already_missing=1
            break
          fi
        done
        if [ "$already_missing" -eq 0 ]; then
          missing+=("$dll")
        fi
      fi
    done < <(objdump -p "$bin" | awk '/DLL Name:/ {print $3}')
  done

  if [ "${#missing[@]}" -gt 0 ]; then
    echo "Missing non-system DLL dependencies:" >&2
    local dll=""
    for dll in "${missing[@]}"; do
      echo "  - $dll" >&2
    done
    return 1
  fi
}

copy_linux_shared_libs() {
  local bin_path="$1"
  local lib_dir="$2"
  mkdir -p "$lib_dir"
  if ! command -v ldd >/dev/null 2>&1; then
    return
  fi

  ldd "$bin_path" | awk '{if ($3 ~ /^\//) print $3}' | sort -u | while IFS= read -r so; do
    local so_name
    so_name="$(basename "$so")"
    case "$so_name" in
      libstdc++.so*|libgcc_s.so*|libglfw.so*|libglfw3.so*)
        cp -f "$so" "$lib_dir/"
        ;;
    esac
  done
}

bundle_macos_glfw() {
  local app_bin="$1"
  local fw_dir="$2"

  if ! command -v otool >/dev/null 2>&1; then
    echo "otool not found; cannot bundle macOS dylibs." >&2
    return 1
  fi
  if ! command -v install_name_tool >/dev/null 2>&1; then
    echo "install_name_tool not found; cannot rewrite macOS dylib paths." >&2
    return 1
  fi

  local glfw_src
  glfw_src="$(otool -L "$app_bin" | awk '/libglfw[.]3[.]dylib/ {print $1; exit}')"
  if [ -z "$glfw_src" ]; then
    return 0
  fi
  if [ ! -f "$glfw_src" ]; then
    echo "GLFW dylib referenced by app was not found on build host: $glfw_src" >&2
    return 1
  fi

  mkdir -p "$fw_dir"
  local glfw_dst="$fw_dir/libglfw.3.dylib"
  cp -f "$glfw_src" "$glfw_dst"
  chmod 755 "$glfw_dst"

  install_name_tool -id "@executable_path/../Frameworks/libglfw.3.dylib" "$glfw_dst"
  install_name_tool -change "$glfw_src" "@executable_path/../Frameworks/libglfw.3.dylib" "$app_bin"
}

generate_macos_icns_from_png() {
  local png_path="$1"
  local out_icns="$2"

  if [ ! -f "$png_path" ]; then
    return 1
  fi
  if ! command -v sips >/dev/null 2>&1; then
    return 1
  fi
  if ! command -v iconutil >/dev/null 2>&1; then
    return 1
  fi

  local iconset_dir
  iconset_dir="$(mktemp -d)"
  mkdir -p "$iconset_dir/app.iconset"
  local set_dir="$iconset_dir/app.iconset"

  sips -z 16 16     "$png_path" --out "$set_dir/icon_16x16.png" >/dev/null
  sips -z 32 32     "$png_path" --out "$set_dir/icon_16x16@2x.png" >/dev/null
  sips -z 32 32     "$png_path" --out "$set_dir/icon_32x32.png" >/dev/null
  sips -z 64 64     "$png_path" --out "$set_dir/icon_32x32@2x.png" >/dev/null
  sips -z 128 128   "$png_path" --out "$set_dir/icon_128x128.png" >/dev/null
  sips -z 256 256   "$png_path" --out "$set_dir/icon_128x128@2x.png" >/dev/null
  sips -z 256 256   "$png_path" --out "$set_dir/icon_256x256.png" >/dev/null
  sips -z 512 512   "$png_path" --out "$set_dir/icon_256x256@2x.png" >/dev/null
  sips -z 512 512   "$png_path" --out "$set_dir/icon_512x512.png" >/dev/null
  cp "$png_path" "$set_dir/icon_512x512@2x.png"

  if ! iconutil -c icns "$set_dir" -o "$out_icns" >/dev/null 2>&1; then
    rm -rf "$iconset_dir"
    return 1
  fi
  rm -rf "$iconset_dir"
  return 0
}

case "$platform" in
  macos)
    stage_dir="$dist_dir/${app_name}-${version}-macos-${arch}"
    bundle_dir="$stage_dir/${app_name}.app"
    rm -rf "$stage_dir"
    mkdir -p "$bundle_dir/Contents/MacOS" "$bundle_dir/Contents/Resources" "$bundle_dir/Contents/Frameworks"

    cp "$binary_path" "$bundle_dir/Contents/MacOS/$app_name"
    chmod +x "$bundle_dir/Contents/MacOS/$app_name"
    bundle_macos_glfw "$bundle_dir/Contents/MacOS/$app_name" "$bundle_dir/Contents/Frameworks"
    cp -R "$asset_dir" "$bundle_dir/Contents/Resources/"
    find "$bundle_dir/Contents/Resources/asset" -name '.DS_Store' -delete

    icon_plist=""
    mac_bundle_icon="$bundle_dir/Contents/Resources/app.icns"
    if [ -f "$mac_icon_icns" ]; then
      cp "$mac_icon_icns" "$mac_bundle_icon"
    elif generate_macos_icns_from_png "$app_icon_png" "$mac_bundle_icon"; then
      :
    fi

    if [ -f "$mac_bundle_icon" ]; then
      icon_plist=$'  <key>CFBundleIconFile</key>\n  <string>app.icns</string>'
    fi

    cat > "$bundle_dir/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleDevelopmentRegion</key>
  <string>en</string>
  <key>CFBundleExecutable</key>
  <string>${app_name}</string>
  <key>CFBundleIdentifier</key>
  <string>com.stratfinder.app</string>
  <key>CFBundleInfoDictionaryVersion</key>
  <string>6.0</string>
  <key>CFBundleName</key>
  <string>${app_name}</string>
  <key>CFBundlePackageType</key>
  <string>APPL</string>
${icon_plist}
  <key>CFBundleShortVersionString</key>
  <string>${version}</string>
  <key>CFBundleVersion</key>
  <string>${version}</string>
  <key>LSMinimumSystemVersion</key>
  <string>11.0</string>
  <key>NSHighResolutionCapable</key>
  <true/>
</dict>
</plist>
PLIST

    if command -v codesign >/dev/null 2>&1; then
      codesign --force --deep --sign - "$bundle_dir" >/dev/null
    fi

    if [ "$archive_output" != "0" ]; then
      (
        cd "$dist_dir"
        zip -qr "${app_name}-${version}-macos-${arch}.zip" "${app_name}-${version}-macos-${arch}"
      )
      echo "Created $dist_dir/${app_name}-${version}-macos-${arch}.zip"
    fi
    ;;

  linux)
    stage_dir="$dist_dir/${app_name}-${version}-linux-${arch}"
    rm -rf "$stage_dir"
    mkdir -p "$stage_dir"

    cp "$binary_path" "$stage_dir/${app_name}.bin"
    chmod +x "$stage_dir/${app_name}.bin"
    copy_linux_shared_libs "$stage_dir/${app_name}.bin" "$stage_dir/lib"
    cat > "$stage_dir/$app_name" <<LAUNCH
#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="\$(cd "\$(dirname "\$0")" && pwd)"
export LD_LIBRARY_PATH="\$ROOT_DIR/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
cd "\$ROOT_DIR"
exec "\$ROOT_DIR/${app_name}.bin" "\$@"
LAUNCH
    chmod +x "$stage_dir/$app_name"
    cp -R "$asset_dir" "$stage_dir/"
    find "$stage_dir/asset" -name '.DS_Store' -delete
    cat > "$stage_dir/${app_name}.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=${app_name}
Exec=./${app_name}
Icon=asset/icons/app.png
Terminal=false
Categories=Utility;
DESKTOP
    chmod +x "$stage_dir/${app_name}.desktop"

    if [ "$archive_output" != "0" ]; then
      tar -C "$dist_dir" -czf "$dist_dir/${app_name}-${version}-linux-${arch}.tar.gz" "${app_name}-${version}-linux-${arch}"
      echo "Created $dist_dir/${app_name}-${version}-linux-${arch}.tar.gz"
    fi
    ;;

  windows)
    stage_dir="$dist_dir/${app_name}-${version}-windows-${arch}"
    rm -rf "$stage_dir"
    mkdir -p "$stage_dir"

    cp "$binary_path" "$stage_dir/${app_name}.exe"
    copy_windows_dlls "$stage_dir/${app_name}.exe" "$stage_dir"
    cp -R "$asset_dir" "$stage_dir/"
    find "$stage_dir/asset" -name '.DS_Store' -delete

    if [ "$archive_output" != "0" ]; then
      (
        cd "$dist_dir"
        zip -qr "${app_name}-${version}-windows-${arch}.zip" "${app_name}-${version}-windows-${arch}"
      )
      echo "Created $dist_dir/${app_name}-${version}-windows-${arch}.zip"
    fi
    ;;

  *)
    echo "Unsupported platform: $platform" >&2
    exit 1
    ;;
esac
