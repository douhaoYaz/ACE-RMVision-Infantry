#include "main_widget.h"
#ifdef GUI
#include "stdafx.h"
#include <QGridLayout>
#include <QDebug>
#include <QDir>

CapturePage::CapturePage(QWidget* parent):
        QWidget(parent) {

    QGridLayout* l = new QGridLayout();

    lbl_cap1 = new QLabel("摄像头1参数：", this);
    lbl_cap1->setFixedSize(100, 30);
    l->addWidget(lbl_cap1, 1, 1);

    lbl_cap2 = new QLabel("摄像头2参数：", this);
    lbl_cap2->setFixedSize(100, 30);
    l->addWidget(lbl_cap2, 2, 1);

    lbl_type = new QLabel("兵种:", this);
    l->addWidget(lbl_type, 3, 1);

    QDir dir("../parameter/capture");
    auto list = dir.entryList({ "*.xml" }, QDir::Files|QDir::Readable, QDir::Name);

    selector_cap1 = new QComboBox(this);
    selector_cap1->setFixedSize(200, 30);
    selector_cap1->addItems(list);
    l->addWidget(selector_cap1, 1, 2);

    selector_cap2 = new QComboBox(this);
    selector_cap2->setFixedSize(200, 30);
    selector_cap2->addItem("");
    selector_cap2->addItems(list);
    l->addWidget(selector_cap2, 2, 2);

    selector_type = new QComboBox(this);
    selector_type->addItems({ "步兵", "哨兵", "英雄", "工程", "无人机", "前哨站" });
    selector_type->setFixedSize(200, 30);
    l->addWidget(selector_type, 3, 2);

    l->setAlignment(Qt::AlignCenter);


    this->setLayout(l);

}

void CapturePage::getParameter(QVariant& cap1, QVariant& cap2, int& type) {
    cap1 = selector_cap1->currentText();
    cap2 = selector_cap2->currentText();
    type = selector_type->currentIndex();
}

VideoPage::VideoPage(QWidget* parent):
        QWidget(parent) {

    QGridLayout* l = new QGridLayout();

    lbl_video = new QLabel("视频：", this);
    lbl_video->setFixedSize(50, 30);
    l->addWidget(lbl_video, 1, 1);

    lbl_fps = new QLabel("帧数：", this);
    lbl_fps->setFixedSize(50, 30);
    l->addWidget(lbl_fps, 2, 1);

    selector = new QComboBox(this);
    selector->setFixedSize(245, 30);
    QDir dir("../video");
    auto list = dir.entryList({ "*.avi", "*.mp4" }, QDir::Files|QDir::Readable, QDir::Name);
    selector->addItems(list);
    l->addWidget(selector, 1, 2);

    cnt_fps = new QSpinBox(this);
    cnt_fps->setFixedSize(60, 30);
    cnt_fps->setMaximum(999);
    l->addWidget(cnt_fps, 2, 2);

    this->setLayout(l);
}

void VideoPage::getParameter(QVariant& video, QVariant& fps) {
    video = selector->currentText();
    fps = cnt_fps->value();
}

StartWidget::StartWidget(QWidget *parent):
        QWidget(parent) {

    this->setFixedSize(350, 350);

    QVBoxLayout* l = new QVBoxLayout();

    QHBoxLayout* l_icon = new QHBoxLayout();

    QLabel* icon = new QLabel(this);
    icon->setFixedSize(50, 50);
    icon->setStyleSheet("image: url(../GUI/logo.png)");
    QLabel* icon_txt = new QLabel(this);
    icon_txt->setFixedSize(150, 50);
    icon_txt->setStyleSheet("image: url(../GUI/txt.png)");
    l_icon->addStretch(1);
    l_icon->addWidget(icon);
    l_icon->addWidget(icon_txt);
    l_icon->addStretch(1);

    l->addStretch(1);
    l->addLayout(l_icon);
    l->addStretch(1);

    widget_tab = new TabWidget(this);
    widget_tab->setFixedSize(350, 180);
    page_cap = new CapturePage(widget_tab);
    widget_tab->add(page_cap, "摄像头模式");
    page_video = new VideoPage(widget_tab);
    widget_tab->add(page_video, "视频模式");
    l->addWidget(widget_tab, 0, Qt::AlignCenter);
    l->addStretch(1);

    btn_start = new IconButton("../GUI/icon/start_light.png", "../GUI/icon/start_white.png", this);
    btn_start->setCursor(Qt::PointingHandCursor);
    btn_start->setFixedSize(50, 50);
    l->addWidget(btn_start, 0, Qt::AlignCenter);
    l->addStretch(1);

    l->setAlignment(Qt::AlignCenter);
    l->setMargin(0);
    this->setLayout(l);

    QObject::connect(btn_start, SIGNAL(clicked()), this, SLOT(onStart()));

}


void StartWidget::onStart() {

    int idx = widget_tab->getCurrentIndex();
    QVariant param1;
    QVariant param2;
    int type = 0;
    if (idx == 0)
        page_cap->getParameter(param1, param2, type);
    else if (idx == 1)
        page_video->getParameter(param1, param2);
    emit openByMode(idx, param1, param2, type);
}
#endif
