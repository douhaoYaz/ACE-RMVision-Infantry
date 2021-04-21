#include "component.h"
#ifdef GUI
#include <QPainter>
#include <QPaintEvent>
#include <QDebug>
#include <QStyleOption>

Button::Button(QWidget* parent): QPushButton(parent) { }

Button::Button(const QString& txt, QWidget* parent):
QPushButton(txt, parent) { }

//IconButton::IconButton(QImage common, QImage hover, QWidget* parent):
//        QPushButton(parent), img_common(common), img_hover(hover), is_hover(false) { }

IconButton::IconButton(const QString& common, const QString& hover,
        QWidget* parent): QPushButton(parent) {
    QString qss("IconButton { image: url(");
    qss += common;
    qss += "); } IconButton::hover { image: url(";
    qss += hover;
    qss += "); }";
    this->setStyleSheet(qss);
}

//void IconButton::enterEvent(QEvent*) {
//    this->is_hover = true;
//}

//void IconButton::leaveEvent(QEvent*) {
//    this->is_hover = false;
//}

void IconButton::paintEvent(QPaintEvent*) {
    QStyleOption opt;
    opt.init(this);
    QPainter* painter = new QPainter(this);
    this->style()->drawPrimitive(QStyle::PE_Widget, &opt, painter, this);
    painter->end();
//    QPainter* painter = new QPainter(this);
//    if (is_hover)
//        painter->drawImage(event->rect(), img_hover, img_hover.rect());
//    else
//        painter->drawImage(event->rect(), img_common, img_hover.rect());
//    painter->end();
}

TitleBar::TitleBar(QWidget* parent):
    QWidget(parent), is_pressing(false) {

    this->setFixedHeight(30);

    icon = new QLabel(this);
    icon->setFixedSize(25, 25);

    title = new QLabel(this);
    title->setFixedHeight(25);
    title->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    btn_minimum = new IconButton("../GUI/icon/minus_light.png",
            "../GUI/icon/minus_white.png", this);
    btn_minimum->setFixedSize(25, 25);

    btn_close = new IconButton("../GUI/icon/close_light.png",
            "../GUI/icon/close_white.png", this);
    btn_close->setFixedSize(25, 25);

    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(icon);
    layout->addStretch(1);
    layout->addWidget(title);
    layout->addStretch(1);
    layout->addWidget(btn_minimum);
    layout->addWidget(btn_close);
    layout->setAlignment(Qt::AlignVCenter);
    layout->setMargin(0);
    this->setLayout(layout);

    connect(btn_minimum, SIGNAL(clicked()), this, SLOT(onMinimum()));
    connect(btn_close, SIGNAL(clicked()), this, SLOT(onClose()));

}

void TitleBar::mousePressEvent(QMouseEvent* event) {
    if (!is_pressing && event->button() == Qt::LeftButton) {
        is_pressing = true;
        pt_start = event->globalPos();
    }
}

void TitleBar::mouseMoveEvent(QMouseEvent* event) {
    if (is_pressing) {
        QWidget* p = this->parentWidget();
        QRect rect = p->geometry();
        p->move(rect.topLeft() + event->globalPos() - pt_start);
        pt_start = event->globalPos();
    }
}

void TitleBar::mouseReleaseEvent(QMouseEvent* event) {
    if (is_pressing && event->button() == Qt::LeftButton)
        is_pressing = false;

}

void TitleBar::paintEvent(QPaintEvent*) {
    QStyleOption opt;
    opt.init(this);
    QPainter* painter = new QPainter(this);
    this->style()->drawPrimitive(QStyle::PE_Widget, &opt, painter, this);
    painter->end();
}

bool TitleBar::eventFilter(QObject* watched, QEvent* event) {
    auto type = event->type();

    if (type == QEvent::WindowStateChange || type == QEvent::Resize) {
        this->setFixedWidth(this->parentWidget()->width());
        return true;
    }
    return QWidget::eventFilter(watched, event);
}

void TitleBar::onMinimum() {
    this->window()->minimumSize();
}

void TitleBar::onClose() {
    this->window()->close();
}

MainWindow::MainWindow(QWidget *parent):
        QFrame(parent) {

    this->setWindowFlags(Qt::FramelessWindowHint);
    QVBoxLayout* layout = new QVBoxLayout();
    titlebar = new TitleBar(this);
    widget = new QWidget(this);
    layout->addWidget(titlebar);
    layout->addWidget(widget);
    layout->setMargin(5);
    this->setLayout(layout);
}

void MainWindow::setWidget(QWidget* widget) {
    QLayout* layout = this->layout();
    widget->setParent(this);
    if (this->widget) {

        this->widget->setParent(nullptr);
        layout->removeWidget(this->widget);
        delete this->widget;
    }
    layout->addWidget(widget);
    this->widget = widget;

}

//void MainWindow::paintEvent(QPaintEvent*) {
//    QStyleOption opt;
//    opt.init(this);
//    QPainter* painter = new QPainter(this);
//    this->style()->drawPrimitive(QStyle::PE_Widget, &opt, painter, this);
//    painter->end();
//}

CollapseItem::CollapseItem(const QString& title, QSize sz, QWidget* parent):
        QWidget(parent), visible(false), sz_btn(sz) {
    QVBoxLayout* layout = new QVBoxLayout;

    btn = new CollapseButton(title, this);
    btn->setFixedSize(sz);

    layout->addWidget(btn);
    setLayout(layout);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);


    connect(btn, SIGNAL(clicked()), this, SLOT(onClick()));

}

void CollapseItem::addItem(QWidget* item) {
    items.push_back(item);
    this->layout()->addWidget(item);
    item->setVisible(visible);
}

void CollapseItem::showOrHide() {
    visible = !visible;
    for (auto iter=items.begin(); iter!=items.end(); ++iter) {
        (*iter)->setVisible(visible);
    }
    this->adjustSize();
}

void CollapseItem::onClick() {
    emit clicked();
}

CollapseBox::CollapseBox(QWidget* parent): QWidget(parent) {
    QVBoxLayout* layout = new QVBoxLayout();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    setLayout(layout);
}

void CollapseBox::addItem(CollapseItem* item) {
    items.append(item);
    this->layout()->addWidget(item);
    connect(item, SIGNAL(clicked()), this, SLOT(onClick()));
}

void CollapseBox::onClick() {
    CollapseItem* itm = (CollapseItem*)sender();
    itm->showOrHide();
    this->adjustSize();
}


CheckableButton::CheckableButton(const QString& txt, QWidget* parent):
        QPushButton(txt, parent) {
    this->setCheckable(true);
    this->setChecked(false);
    this->setCursor(Qt::PointingHandCursor);
}

TabWidget::TabWidget(QWidget* parent): QWidget(parent) {

    QVBoxLayout* l = new QVBoxLayout();

    hlayout_btn = new QHBoxLayout();
    l->addLayout(hlayout_btn);

    gp_tab = new QButtonGroup();

    pages = new QStackedWidget(this);
    l->addWidget(pages);

    l->setAlignment(Qt::AlignCenter);
    l->setMargin(0);
    this->setLayout(l);

    connect(gp_tab, SIGNAL(buttonClicked(int)), this, SLOT(switchPage(int)));

}

void TabWidget::add(QWidget* widget, const QString& txt_btn) {
    CheckableButton* btn = new CheckableButton(txt_btn, this);
    int cnt = pages->count();
    if (cnt == 0)
        btn->setChecked(true);

    int width = this->width();
    auto btns = gp_tab->buttons();

    cnt++;
    for (auto iter=btns.begin(); iter!=btns.end(); ++iter)
        (*iter)->setFixedWidth(width / cnt);
    btn->setFixedWidth(width/cnt);
    hlayout_btn->addWidget(btn);
    hlayout_btn->setAlignment(Qt::AlignVCenter);
    hlayout_btn->setMargin(0);
    hlayout_btn->setSpacing(0);
    gp_tab->addButton(btn, pages->count());
    pages->addWidget(widget);


}

void TabWidget::switchPage(int idx) {
    pages->setCurrentIndex(idx);
}

CollapseButton::CollapseButton(const QString& txt, QWidget* parent):
        QPushButton(txt, parent) {
    this->setCheckable(true);
    this->setChecked(false);
    this->setCursor(Qt::PointingHandCursor);
}
#endif
