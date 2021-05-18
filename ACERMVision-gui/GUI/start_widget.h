#include "stdafx.h"
#ifdef GUI
#ifndef START_WIDGET_H
#define START_WIDGET_H

#include "component.h"
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QFile>
#include <QVariant>
#include <QMouseEvent>

class CapturePage: public QWidget {
    Q_OBJECT
public:
    CapturePage(QWidget* parent=nullptr);

    void getParameter(QVariant& cap1, QVariant& cap2, int& type);

private:
    QLabel* lbl_cap1;
    QLabel* lbl_cap2;
    QLabel* lbl_type;
    QComboBox* selector_cap1;
    QComboBox* selector_cap2;
    QComboBox* selector_type;
};

class VideoPage: public QWidget {
    Q_OBJECT
public:
    VideoPage(QWidget* parent=nullptr);

    void getParameter(QVariant& video, QVariant& fps);

private:
    QLabel* lbl_video;
    QLabel* lbl_fps;
    QComboBox* selector;
    QSpinBox* cnt_fps;
};

class StartWidget: public QWidget {
    Q_OBJECT
public:
    explicit StartWidget(QWidget* parent=nullptr);

private:
    QLabel* icon;

    TabWidget* widget_tab;
    CapturePage* page_cap;
    VideoPage* page_video;

    IconButton* btn_start;

private slots:
    void onStart();

signals:
    void openByMode(int, QVariant, QVariant, int);
};

#endif // START_WIDGET_H
#endif
