#ifndef UI_TOOL_H
#define UI_TOOL_H

#include "stdafx.h"
#include "utils.h"
#if defined(GUI)
#include "GUI/main_widget.h"
class UITool {
public:
    UITool()=delete;

    static void log(const char* txt, int color=FONT_NONE);
    static void log(const std::string& txt) {
        log(txt.c_str());
    }

    template <typename _Type1, typename _Type2>
    static void log2(_Type1 key, _Type2 val, bool is_show_val=true) {
#if defined(GUI)
        MainWidget::log2(Tool::toString(key), Tool::toString(val), is_show_val);
#elif defined(CUI)
#else
#endif
    }

    static void show();
};

#endif //GUI

#endif // UI_TOOL_H
