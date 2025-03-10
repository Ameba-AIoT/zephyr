# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

target_link_libraries(app PRIVATE wifi_libs)
zephyr_library_sources_ifdef(CONFIG_WIFI_AMEBA
  src/ameba_wifi_drv.c
  )
