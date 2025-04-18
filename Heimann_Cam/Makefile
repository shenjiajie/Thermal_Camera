# # 编译器
# # CC := aarch64-linux-gnu-gcc
# CC := /opt/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc

# # 源码目录
# SRCDIR := .

# # 查找所有源文件
# SRCS := $(wildcard $(SRCDIR)/*.c)

# # 自动生成的依赖文件（*.d）
# DEPS := $(SRCS:.c=.d)

# # 输出文件
# TARGET := app

# # 编译参数
# CFLAGS := -Wall -O2 -MMD -MP
# LDFLAGS := -static  # 添加静态链接选项

# # 默认目标
# all: $(TARGET)

# # 编译规则（同时生成 .d 依赖文件）
# $(TARGET): $(SRCS)
# # $(CC) $(CFLAGS) $^ -o $@
# 	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@ -lm


# # 包含自动生成的 .d 文件（如果存在）
# -include $(DEPS)

# # 清理
# clean:
# 	rm -f app *.o *.out *.d *.d.* core

# 交叉编译器路径
CC := /opt/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc

# 系统路径
SYSROOT := /opt/sysroot_debian11
SDL2_PATH := /opt/sdl2-rk3588-install

# 源码目录
SRCDIR := .
TARGET := app
# 查找所有源文件
SRC := $(wildcard $(SRCDIR)/*.c)

# 静态/动态链接控制
LINK ?= dynamic

# 编译选项
CFLAGS += -I$(SDL2_PATH)/include --sysroot=$(SYSROOT) -Wall -O2 -MMD -MP
LDFLAGS += -L$(SDL2_PATH)/lib -Wl,-rpath-link=$(SYSROOT)/usr/lib/aarch64-linux-gnu

ifeq ($(LINK),static)
    LDFLAGS += -static
endif

LDLIBS := -lSDL2 -lgbm -lm

# 默认目标
all: $(TARGET)

# 编译规则
$(TARGET): $(SRC)
	@echo "[CC]  $@"
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)


# 自动依赖
-include $(SRC:.c=.d)

# 清理
clean:
	@echo "[CLEAN]"
	rm -f *.o *.d $(TARGET)

