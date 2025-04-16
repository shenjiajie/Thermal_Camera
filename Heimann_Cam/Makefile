# 编译器
CC := /opt/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc

# 源码目录
SRCDIR := .

# 查找所有源文件
SRCS := $(wildcard $(SRCDIR)/*.c)

# 自动生成的依赖文件（*.d）
DEPS := $(SRCS:.c=.d)

# 输出文件
TARGET := app

# 编译参数
CFLAGS := -Wall -O2 -MMD -MP
LDFLAGS := -static  # 添加静态链接选项

# 默认目标
all: $(TARGET)

# 编译规则（同时生成 .d 依赖文件）
$(TARGET): $(SRCS)
# $(CC) $(CFLAGS) $^ -o $@
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@ -lm


# 包含自动生成的 .d 文件（如果存在）
-include $(DEPS)

# 清理
clean:
	rm -f app *.o *.out *.d *.d.* core
