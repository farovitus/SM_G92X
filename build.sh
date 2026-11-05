#!/usr/bin/env bash
#
# Build the SM-G920F (zeroflte) 6.6 kernel on macOS with Android clang.
#
#   ./build.sh --zip            Image, DTB and an artifact ZIP
#   ./build.sh --config-only    write .config only (no compilation)
#   ./build.sh --clean ...      remove the previous output first
#
# Environment: KERNEL_OUT, PRODUCT_OUT, JOBS, USE_CCACHE (default 1),
# LLVM_BIN (default: Android Studio NDK 27.1 clang 18), MAKE.

set -euo pipefail

kernel_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
build_root_dir=$(dirname "$kernel_dir")

device_model=${DEVICE_MODEL:-G920F}
base_defconfig=gki_defconfig
board_fragment="$kernel_dir/arch/arm64/configs/zeroflte_gki.fragment"
dtb_name=${DTB_NAME:-exynos/exynos7420-zeroflte.dtb}

product_out=${PRODUCT_OUT:-"$build_root_dir/kernel_out/ack66"}
kernel_out=${KERNEL_OUT:-"$product_out/KERNEL_OBJ"}
tools_dir=${TOOLS_DIR:-"$build_root_dir/tools"}
llvm_bin=${LLVM_BIN:-"$HOME/Library/Android/sdk/ndk/27.1.12297006/toolchains/llvm/prebuilt/darwin-x86_64/bin"}
llvm_major=${LLVM_MAJOR:-18}
openssl_dir=${OPENSSL_DIR:-/opt/homebrew/opt/openssl@3}
make_cmd=${MAKE:-gmake}
jobs=${JOBS:-$(sysctl -n hw.ncpu)}
use_ccache=${USE_CCACHE:-1}
ccache_config_path=${CCACHE_CONFIGPATH:-"$kernel_dir/ccache.conf"}
ccache_dir=${CCACHE_DIR:-"$build_root_dir/.ccache"}
ccache_temp_dir=${CCACHE_TEMPDIR:-"$ccache_dir/tmp"}

clean_build=0
mode=""

usage() {
	echo "usage: $0 [--clean] (--zip | --config-only)"
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--clean) clean_build=1 ;;
		--zip) mode=zip ;;
		--config-only) mode=config ;;
		-h|--help) usage; exit 0 ;;
		*) usage >&2; exit 2 ;;
	esac
	shift
done
[[ -n "$mode" ]] || { usage >&2; exit 2; }

validate_output_path() {
	local p=$1

	if [[ -z "$p" || "$p" != /* ]]; then
		echo "error: build output path must be absolute: $p" >&2
		exit 1
	fi
	case "$p" in
		/|*'/../'*|*/..|*'/./'*|*/.|"$build_root_dir"|"$kernel_dir"|"$kernel_dir"/*)
			echo "error: unsafe build output path: $p" >&2
			exit 1
			;;
	esac
}

# The tree holds files that differ only by case (xt_mark.h, xt_MARK.h).
# A case-insensitive filesystem merges them and breaks the build.
require_case_sensitive() {
	local dir=$1
	local probe="$dir/.nk7420-case-probe.$$"

	mkdir -p "$dir"
	: > "$probe"
	if [[ -e "$dir/.NK7420-CASE-PROBE.$$" ]]; then
		rm -f "$probe"
		echo "error: $dir is not on a case-sensitive filesystem" >&2
		exit 2
	fi
	rm -f "$probe"
}

validate_output_path "$kernel_out"
validate_output_path "$product_out"
require_case_sensitive "$kernel_dir"
require_case_sensitive "$kernel_out"
require_case_sensitive "$product_out"

# Kbuild scripts expect GNU sed and make; Homebrew installs them under gnubin.
for gnubin in /opt/homebrew/opt/gnu-sed/libexec/gnubin /opt/homebrew/opt/make/libexec/gnubin; do
	[[ -d "$gnubin" ]] && PATH="$gnubin:$PATH"
done
export PATH
sed --version 2>/dev/null | grep -q 'GNU sed' || { echo "error: GNU sed required (brew install gnu-sed)" >&2; exit 1; }
command -v "$make_cmd" >/dev/null || { echo "error: $make_cmd not found (GNU make >= 3.82)" >&2; exit 1; }
[[ -x "$llvm_bin/clang" && -x "$llvm_bin/ld.lld" ]] ||
	{ echo "error: clang/ld.lld not found in $llvm_bin" >&2; exit 1; }
clang_major=$("$llvm_bin/clang" -dM -E -x c /dev/null | sed -n 's/^#define __clang_major__ //p')
if [[ "$clang_major" != "$llvm_major" ]]; then
	echo "error: clang $clang_major found, LLVM $llvm_major expected (set LLVM_MAJOR to override)" >&2
	exit 1
fi

if [[ "$clean_build" == 1 ]]; then
	rm -rf -- "$kernel_out" "$product_out"
	mkdir -p "$kernel_out" "$product_out"
fi

make_args=(
	-C "$kernel_dir" O="$kernel_out" -j"$jobs"
	ARCH=arm64 LLVM="$llvm_bin/" LLVM_IAS=1
	# Host tools need the macOS SDK, which only Apple's clang provides.
	HOSTCC=/usr/bin/clang HOSTCXX=/usr/bin/clang++
	# macOS lacks elf.h and friends; take them from the NDK sysroot, after
	# the SDK headers.
	# OpenSSL (sign-file, certs) comes from Homebrew.
	HOSTCFLAGS="-I$tools_dir/macos-host-include -I$openssl_dir/include -idirafter $llvm_bin/../sysroot/usr/include/aarch64-linux-android -idirafter $llvm_bin/../sysroot/usr/include"
	HOSTLDFLAGS="-L$openssl_dir/lib"
)
if [[ "$use_ccache" == 1 ]]; then
	command -v ccache >/dev/null || { echo "error: USE_CCACHE=1 but ccache is unavailable" >&2; exit 1; }
	[[ -f "$ccache_config_path" ]] || { echo "error: missing $ccache_config_path" >&2; exit 1; }
	require_case_sensitive "$ccache_dir"
	mkdir -p "$ccache_temp_dir"
	export CCACHE_CONFIGPATH="$ccache_config_path" CCACHE_DIR="$ccache_dir" CCACHE_TEMPDIR="$ccache_temp_dir"
	make_args+=(CC="ccache $llvm_bin/clang")
fi

# A fixed banner (user@host in /proc/version) instead of the build machine's
# login and hostname, and the commit's date as the build date, so the same
# commit always builds the same kernel.
export KBUILD_BUILD_USER=${KBUILD_BUILD_USER:-FAROVITUS}
export KBUILD_BUILD_HOST=${KBUILD_BUILD_HOST:-nk7420}
export KBUILD_BUILD_TIMESTAMP=${KBUILD_BUILD_TIMESTAMP:-$(git -C "$kernel_dir" log -1 --format=%cd --date=rfc2822)}

kmake() {
	"$make_cmd" "${make_args[@]}" "$@"
}

# gki_defconfig plus the board fragment. Fail if a fragment symbol did not
# keep its value, so a missing dependency cannot pass silently.
configure() {
	kmake "$base_defconfig"
	# Kconfig keeps the last assignment of a symbol, so appending the
	# fragment merges it (merge_config.sh needs GNU sed).
	cat "$board_fragment" >> "$kernel_out/.config"
	kmake olddefconfig

	local line sym want bad=0
	while IFS= read -r line; do
		case "$line" in
			CONFIG_*=*) sym=${line%%=*}; want=$line ;;
			"# CONFIG_"*" is not set") sym=${line#\# }; sym=${sym%% *}; want=$line ;;
			*) continue ;;
		esac
		if ! grep -qxF -- "$want" "$kernel_out/.config"; then
			echo "error: $sym: wanted '$want', got '$(grep -E "^(# )?$sym[= ]" "$kernel_out/.config" || echo unset)'" >&2
			bad=1
		fi
	done < "$board_fragment"
	[[ "$bad" == 0 ]] || exit 1
}

configure
[[ "$mode" == config ]] && { echo "Config: $kernel_out/.config"; exit 0; }

# The port's boot diagnostics (init/dsh_debug.c) carry a build ID that the
# loader matches against the Image. Derive it from HEAD and rewrite the
# header only when it changes, so it does not force a rebuild.
build_id_h="$kernel_out/include/generated/dsh_build_id.h"
build_id="#define DSH_BUILD_ID 0x$(git -C "$kernel_dir" rev-parse HEAD | cut -c1-16)UL"
mkdir -p "$(dirname "$build_id_h")"
if [[ ! -f "$build_id_h" || "$(<"$build_id_h")" != "$build_id" ]]; then
	printf '%s\n' "$build_id" > "$build_id_h"
fi

kmake Image "$dtb_name" modules

# Modules are signed with this build's key and carry its release string,
# so they only load on the Image built with them. Install them signed and
# stripped, flatten them for /vendor/lib/modules, and write the
# modules.dep that Android's modprobe reads (macOS has no depmod).
mod_stage="$product_out/modules-install"
mod_dir="$product_out/modules"
rm -rf -- "$mod_stage" "$mod_dir"
mkdir -p "$mod_dir"
kmake modules_install INSTALL_MOD_PATH="$mod_stage" INSTALL_MOD_STRIP=1 >/dev/null
find "$mod_stage" -name '*.ko' -exec cp {} "$mod_dir/" \;
rm -rf -- "$mod_stage"
mod_deps() {
	"$llvm_bin/llvm-objcopy" --dump-section .modinfo="$mod_dir/.modinfo" "$mod_dir/$1.ko" /dev/null
	tr '\0' '\n' < "$mod_dir/.modinfo" | sed -n 's/^depends=//p' | tr ',' '\n' | sed '/^$/d'
	rm -f -- "$mod_dir/.modinfo"
}
# Each line lists every module the first needs, nearest dependency first.
mod_closure() {
	local d
	for d in $(mod_deps "$1"); do
		mod_closure "$d"
		echo "$d"
	done
}
: > "$mod_dir/modules.dep"
for ko in "$mod_dir"/*.ko; do
	m=$(basename "$ko" .ko)
	deps=$(mod_closure "$m" | awk '!seen[$0]++' | tail -r | sed 's/$/.ko/' | tr '\n' ' ')
	deps=${deps% }
	# Like depmod: no space after the colon when there are no dependencies.
	# Android's modprobe reads a trailing space as an empty module name.
	echo "$m.ko:${deps:+ $deps}" >> "$mod_dir/modules.dep"
done

image="$kernel_out/arch/arm64/boot/Image"
dtb="$kernel_out/arch/arm64/boot/dts/$dtb_name"

# S-Boot loads the DTB from a DTBH table and picks it by the model_info-*
# properties. Build the table from this one DTB only.
dt_image="$product_out/boot.img-dtb"
dt_stage="$product_out/dt-stage"
rm -rf -- "$dt_stage"
mkdir -p "$dt_stage"
cp "$dtb" "$dt_stage/"
"$tools_dir/dtbTool" -s 2048 -d "$dt_stage/" -o "$dt_image" >/dev/null
rm -rf -- "$dt_stage"
cp "$image" "$product_out/boot.img-zImage"
release=$(<"$kernel_out/include/config/kernel.release")
zip_name="SM-$device_model-$release-artifacts.zip"
stage="$product_out/stage"

rm -rf -- "$stage"
mkdir -p "$stage"
cp "$image" "$stage/Image"
cp "$dtb" "$stage/$(basename "$dtb_name")"
cp "$kernel_out/.config" "$stage/config"
cp "$kernel_out/System.map" "$stage/System.map"
cp "$dt_image" "$stage/boot.img-dtb"
cp -R "$mod_dir" "$stage/modules"
rm -f -- "$product_out/$zip_name"
(cd "$stage" && zip -qrX "$product_out/$zip_name" Image "$(basename "$dtb_name")" boot.img-dtb config System.map modules)
(cd "$product_out" && shasum -a 256 "$zip_name" > "$zip_name.sha256")
rm -rf -- "$stage"

echo "Kernel: $release"
echo "Image:  $image"
echo "DTB:    $dtb"
echo "Modules: $mod_dir"
echo "ZIP:    $product_out/$zip_name (artifacts only, not flashable)"
