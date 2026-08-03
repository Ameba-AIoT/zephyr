# RTK Ameba 自定义 Linker Script 说明

## 背景

RTK Ameba SDK 当前使用以下工具链：

| 工具链 | ld.bfd 版本 | 受影响的 SoC |
|--------|------------|------------|
| asdk-10.3.1 | 2.35.2 | AmebaDplus / AmebaD / AmebaSmart |
| asdk-12.3.1 | 2.38.20 | AmebaG2 |

这两个版本的 `ld.bfd` 在与 Zephyr v4.4.1 的标准链接脚本配合时存在若干 bug，
导致以下构建失败。

**上述 bug 在 ld.bfd 2.42（asdk-14.3.1）中均已修复。**

---

## Bug 1：`.intList*` 孤立节被放入 FLASH（所有 SoC）

### 机制

Zephyr 的 ISR 描述符存放在 `.intList*` 节中，链接脚本要求将其放入独立的
`IDT_LIST` 内存区域（VMA=`0xFFFF8000`），以供 `gen_isr_tables.py` 脚本读取。

Zephyr v4.4.1 的标准 `linker.ld` 中，`.intList` 的声明规则通过
`snippets-sections.ld` 注入，该文件在 `SECTIONS` 块的末段（RAM 区之后）才被
`#include`：

```
SECTIONS {
    ...
    GROUP_START(ROMABLE_REGION)   ← 第 112 行（v4.4.1）
        rom_start: { ... }
        .text: { ... }
        ...
    GROUP_END(ROMABLE_REGION)     ← 第 273 行
    ...
    #include <snippets-sections.ld>  ← 第 394 行，此处包含 intlist.ld
    ...
}
```

ld.bfd 2.35/2.38 的孤立节放置算法在**扫描到 `snippets-sections.ld` 之前**，
就已经处理了来自各对象文件的 `.intList*` 输入节，将其归入当时最近的
`ROMABLE_REGION` GROUP，即 **FLASH 区域**。

### 症状

构建在 `gen_isr_tables.py` 阶段终止：

```
gen_isr_tables.py: error: Cannot find the intlist section!
```

**根本原因**：`.intList` 节被放入 FLASH（如地址 `0x0e004454`），而不是
`IDT_LIST`（`0xFFFF8000`）。

正确情况下，`zephyr_pre0.map` 中应可见：

```
.intList        0x00000000ffff8000      0x118
 .intList       0xffff8008       0x10  zephyr/libzephyr.a(soc.c.obj)
```

发生 bug 时，`.intList` 输入节出现在 FLASH 地址段中（如 `0x0e0xxxxx`），
最终 `.intList` 输出节为空，ELF 中找不到该节。

---

## Bug 2：DWARF debug 节孤立节放入 FLASH，二进制膨胀（所有 SoC）

### 机制

同 Bug 1，ld.bfd 的孤立节放置算法会在 `snippets-sections.ld` 和
`debug-sections.ld`（在链接脚本末尾，专门处理 debug 节）处理之前，
将来自 `libgcc.a` 等第三方库的 `.debug_info`、`.debug_str`、
`.debug_frame` 等 DWARF 节归入 `ROMABLE_REGION` GROUP（FLASH）。

### 症状

构建失败，FLASH overflow：

```
ld.bfd: region 'FLASH' overflowed by 3749152 bytes
```

`zephyr_pre0.map` 中可见大量 debug 节出现在 FLASH 地址段（`0x0e0xxxxx`）：

```
.debug_info    0x000000000e7934db       0x26  libgcc.a(_arm_fixdfsi.o)
.debug_str     0x000000000e793515      0x12b  libgcc.a(_arm_fixdfsi.o)
```

这使 `rom_start` 节的尺寸虚增至 7+ MB，触发 overflow 检查。

---

## Bug 3：`sw_isr_table` 通配符吸收 `.ARM.attributes`（ld 2.35 + Cortex-A32 / amebasmart）

### 机制

`common-ram.ld` 中 `sw_isr_table` 节使用了 `*()` 通配符，会吸收所有尚未被
声明的节。ld.bfd 2.35 在处理 Cortex-A32 目标文件时，会将 `.ARM.attributes`
节（ARM ELF 工具链属性）放置到 `sw_isr_table` 中，导致该输出节的 ELF 类型
由 `SHT_PROGBITS` 变为 `SHT_ARM_ATTRIBUTES`。

`gen_isr_tables.py` 使用 pyelftools 解析 ELF 时无法处理 `SHT_ARM_ATTRIBUTES`
类型的节，抛出异常。

### 症状

构建失败，`sw_isr_table` 巨大，RAM overflow：

```
ld.bfd: zephyr/zephyr_pre0.elf section 'sw_isr_table' will not fit in region 'RAM'
ld.bfd: region 'RAM' overflowed by 3215360 bytes
```

---

## Bug 4：`SORT()` 与 `.ARM.exidx` 混用报错（ld 2.35 + amebad/amebadplus/amebasmart）

### 机制

Zephyr `initlevel` 相关节使用 `SORT()` 进行有序排列，而 libgcc 中的
`_udivmoddi4.o` 含有 `.ARM.exidx` 节（C++ 异常 unwind 表）。ld.bfd 2.35
不允许同一 GROUP 内同时出现 `SORT()` 有序节和无序节。

### 症状

构建失败：

```
ld.bfd: error: rom_start has both ordered and unordered sections
```

### 解决方案

在 `snippets-sections.ld` 中（通过 `zephyr_linker_sources(SECTIONS ...)` 注入）
于 `ROMABLE_REGION GROUP` 之前 discard `.ARM.exidx*` 节：

```ld
/* soc/realtek/ameba/amebad/discard_exidx.ld */
/DISCARD/ : { *(.ARM.exidx*) *(gnu.linkonce.armexidx.*) }
```

此 workaround 已在各 SoC 的 `CMakeLists.txt` 中通过
`zephyr_linker_sources(SECTIONS ...)` 激活（见 `amebad/`、`amebadplus/`、
`amebasmart/` 的 `CMakeLists.txt`）。

---

## 参考：zephyr_ctc 团队的做法

`~/sdk_backup/zephyr_ctc/zephyr`（Zephyr v4.3.0 + asdk-14.3.1 / ld.bfd 2.42）：

```cmake
# soc/realtek/sheipa_arm/sheipa3_cortex_a32/CMakeLists.txt
set(SOC_LINKER_SCRIPT
    ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld
    CACHE INTERNAL "")
```

**直接使用上游标准 `linker.ld`，零修改。**
因为 ld.bfd 2.42 已修复孤立节放置 bug，不需要任何 workaround。

---

## 我们的 Workaround 方案（Bug 1、2、3）

**原则：不修改任何上游 Zephyr 文件。**

利用 Zephyr 的 `SOC_LINKER_SCRIPT` 机制，让每个 RTK SoC 指向本目录中的
自定义 linker 脚本：

| 文件 | 基于 | 解决 Bug | 使用 SoC |
|------|------|---------|---------|
| `linker_cortex_m.ld` | v4.4.1 `cortex_m/linker.ld` | Bug 1 + Bug 2 | amebad / amebadplus / amebag2 |
| `linker_cortex_a_r.ld` | v4.4.1 `cortex_a_r/linker.ld` | Bug 1 + Bug 3 | amebasmart |

### `linker_cortex_m.ld` 相对 v4.4.1 的改动

在 `SECTIONS {` 开头（`rel-sections.ld` 之前）添加：

```ld
/* 1. intlist 提前：防止 .intList* 被作为孤立节放入 FLASH */
#include <zephyr/linker/intlist.ld>

/* 2. debug 节提前 DISCARD：防止被作为孤立节放入 FLASH */
/DISCARD/ : {
    *(.debug_info) *(.gnu.linkonce.wi.*)
    *(.debug_abbrev) *(.debug_aranges) *(.debug_ranges)
    *(.debug_line) *(.debug_line.*) *(.debug_line_end)
    *(.debug_str) *(.debug_loc) *(.debug_loclists)
    *(.debug_frame) *(.debug_macinfo) *(.debug_macro)
    ... （完整列表见文件本身）
    *(.ARM.attributes)
}
```

### `linker_cortex_a_r.ld` 相对 v4.4.1 的改动

1. 在 `SECTIONS {` 开头添加 intlist 提前（同上）
2. 在 `#include <zephyr/linker/common-ram.ld>` 之前添加：

```ld
/* .ARM.attributes 提前：防止被 sw_isr_table 通配符吸收 */
SECTION_PROLOGUE(.ARM.attributes, 0,)
{
    KEEP(*(.ARM.attributes))
    KEEP(*(.gnu.attributes))
}
```

### 各 SoC CMakeLists.txt 改动

```cmake
# amebad / amebadplus / amebag2
set(SOC_LINKER_SCRIPT
    ${CMAKE_CURRENT_LIST_DIR}/../common/linker_cortex_m.ld
    CACHE INTERNAL "")

# amebasmart
set(SOC_LINKER_SCRIPT
    ${CMAKE_CURRENT_LIST_DIR}/../common/linker_cortex_a_r.ld
    CACHE INTERNAL "")
```

### 上游文件状态

以下文件与 Zephyr v4.4.1 upstream **完全一致，零修改**：

- `include/zephyr/arch/arm/cortex_m/scripts/linker.ld`
- `include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld`

---

## 升级工具链后的处理（asdk-14.3.1 / ld.bfd 2.42）

升级后孤立节 bug 消失，可恢复标准做法（与 zephyr_ctc 一致）：

1. 删除 `linker_cortex_m.ld` 和 `linker_cortex_a_r.ld`
2. 各 SoC CMakeLists.txt 改回标准路径：

   ```cmake
   # amebad / amebadplus / amebag2
   set(SOC_LINKER_SCRIPT
       ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld
       CACHE INTERNAL "")

   # amebasmart
   set(SOC_LINKER_SCRIPT
       ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld
       CACHE INTERNAL "")
   ```

3. 可同时移除各 SoC `CMakeLists.txt` 中的 `discard_exidx.ld` 引用
   及对应的 `.ld` 文件（Bug 4 workaround）

---

## 验证

所有 workaround 均在以下配置下通过完整构建验证：

| 板子 | SoC | 工具链 | 测试应用 |
|------|-----|--------|--------|
| rtl8721f_evb | AmebaG2 (Cortex-M55) | asdk-12.3.1 (ld 2.38) | applications/thread + hello_world |
| rtl872xda_evb | AmebaDplus (Cortex-M55) | asdk-10.3.1 (ld 2.35) | applications/thread + hello_world |
| rtl872xd_evb | AmebaD (Cortex-M33) | asdk-10.3.1 (ld 2.35) | applications/thread + hello_world |
| rtl8730e_evb | AmebaSmart (Cortex-A32) | asdk-10.3.1 (ld 2.35) | applications/thread + hello_world |
