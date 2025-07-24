# arm-toolchain.cmake

# 1. 指定系统名称和架构
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 2. 指定交叉编译器
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 3. 指定目标根文件系统（sysroot）
set(CMAKE_SYSROOT /srv/chroot/ubuntu-arm64/)

# 4. 查找路径设置，优先从 sysroot 查找头文件和库，防止误用主机路径
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)   # 不在 sysroot 查找可执行程序（只用主机上的）
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)    # 仅在 sysroot 查找库文件
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)    # 仅在 sysroot 查找头文件
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)    # 仅在 sysroot 查找 CMake 包

# 5. 设置 sysroot 编译、链接标志
set(CMAKE_C_FLAGS "--sysroot=${CMAKE_SYSROOT}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "--sysroot=${CMAKE_SYSROOT}" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "--sysroot=${CMAKE_SYSROOT}" CACHE STRING "" FORCE)

set(cmake_cxx_try_compile "IGNORE" CACHE BOOL "" FORCE)
set(cmake_c_try_compile "IGNORE" CACHE BOOL "" FORCE)

set(CMAKE_CROSSCOMPILING TRUE CACHE BOOL "Cross compile mode" FORCE)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY" CACHE STRING "" FORCE)
# 6. 强制 Disable TRY_RUN 测试，避免在主机上运行 aarch64 二进制
set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)

set(PYTHON_EXECUTABLE ${CMAKE_SYSROOT}/usr/bin/python3.10)
# 7. 设置 Python 库和头文件路径，指向 sysroot 中对应路径，防止使用宿主机 Python
set(PYTHON_LIBRARY "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.10.so" CACHE FILEPATH "Path to python lib" FORCE)
set(PYTHON_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/include/python3.10" CACHE PATH "Path to python include" FORCE)

# 8. 设置 Python3 查找缓存（某些 find_package 可能使用）
set(Python3_LIBRARY_RELEASE "${PYTHON_LIBRARY}" CACHE FILEPATH "" FORCE)
set(Python3_INCLUDE_DIR "${PYTHON_INCLUDE_DIR}" CACHE PATH "" FORCE)

# 9. 扩展 CMAKE_PREFIX_PATH，方便 find_package 查找 sysroot 内的 cmake 包
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake")

# 10. 设置 pkg-config 环境变量，确保 pkg-config 在 sysroot 查找包配置
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
set(ENV{PKG_CONFIG_LIBDIR}
    "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig"
)
