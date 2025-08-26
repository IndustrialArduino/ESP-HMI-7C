SET(SOURCES ui_Screen1.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_empty_gauge_png.c
    ui_img_full_gauge_png.c
    fonts/ui_font_Times40.c)

add_library(ui ${SOURCES})
