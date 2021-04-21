#include "ui_tool.h"

#ifdef GUI
void UITool::log(const char* txt, int color) {
#ifdef GUI
    MainWidget::log(txt, color);
#else
    WindowPainter::painter << txt << WindowPainter::end;
#endif
}
#endif
