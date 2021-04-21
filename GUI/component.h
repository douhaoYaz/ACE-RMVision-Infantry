#ifdef GUI
#ifndef COMPONENT_H
#define COMPONENT_H

#include "stdafx.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QToolBox>
#include <QVariant>
#include <QPushButton>
#include <iostream>
#include <QWidget>
#include <QButtonGroup>
#include <QStackedWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QPropertyAnimation>

class Button: public QPushButton {
    Q_OBJECT
public:
    explicit Button(QWidget* parent=nullptr);
    explicit Button(const QString& txt, QWidget* parent=nullptr);
};

class IconButton: public QPushButton {
    Q_OBJECT
public:
    //IconButton(QImage common, QImage hover, QWidget* parant=nullptr);
    IconButton(const QString& common, const QString& hover={},
            QWidget* parant=nullptr);

//这样画出来有锯齿
protected:  //@override

    //void enterEvent(QEvent* event) override;

    //void leaveEvent(QEvent* event) override;

    void paintEvent(QPaintEvent* event) override;


protected:
    //QImage img_common;
    //QImage img_hover;
    //bool is_hover;
};

class TitleBar: public QWidget {
    Q_OBJECT
public:
    explicit TitleBar(QWidget* parent=nullptr);

protected:  //@override

    /**
     * @brief 鼠标按下事件处理函数
     * @param event，参数，事件
     */
    virtual void mousePressEvent(QMouseEvent* event) override;
    /**
     * @brief 鼠标移动事件处理函数
     * @param event，参数，事件
     */
    virtual void mouseMoveEvent(QMouseEvent* event) override;
    /**
     * @brief 鼠标释放事件处理函数
     * @param event，参数，事件
     */
    virtual void mouseReleaseEvent(QMouseEvent* event) override;

    virtual void paintEvent(QPaintEvent *event) override;

    virtual bool eventFilter(QObject* watched, QEvent* event) override;

protected slots:

    void onMinimum();

    void onClose();

protected:
    bool is_pressing;
    QPoint pt_start;

    IconButton* btn_close;
    IconButton* btn_minimum;
    QLabel* title;
    QLabel* icon;
};

class MainWindow: public QFrame {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent=nullptr);

    void setWidget(QWidget* widget);

protected:
    QWidget* widget;
    TitleBar* titlebar;
};

class CheckableButton: public QPushButton {
    Q_OBJECT
public:
    explicit CheckableButton(const QString& txt, QWidget* parent=nullptr);

protected:
    using QPushButton::setCheckable;

};

class TabWidget: public QWidget {
    Q_OBJECT
public:
    explicit TabWidget(QWidget* parent=nullptr);

    void add(QWidget* widget, const QString& txt_btn);

    int getCurrentIndex() {
        return pages->currentIndex();
    }

public slots:
    void switchPage(int);


protected:
    QButtonGroup* gp_tab;
    QStackedWidget* pages;

    QHBoxLayout* hlayout_btn;
};

class CollapseButton: public QPushButton {
    Q_OBJECT
public:
    explicit CollapseButton(const QString& txt, QWidget* parent=nullptr);

protected:
    using QPushButton::setCheckable;

};

class CollapseItem: public QWidget {
    Q_OBJECT
public:
    CollapseItem(const QString& title, QSize sz, QWidget* parent=nullptr);
    CollapseItem(const QString& title, int w, int h,
            QWidget* parent=nullptr):
            CollapseItem(title, QSize(w, h), parent) { }


    void addItem(QWidget* item);

    void showOrHide();

signals:
    void clicked();

protected slots:
    void onClick();

protected:
    CollapseButton* btn;
    QVector<QWidget*> items;
    bool visible;
    QSize sz_btn;
};

class CollapseBox: public QWidget {
    Q_OBJECT
public:

    explicit CollapseBox(QWidget* parent=nullptr);

    void addItem(CollapseItem* item);

protected slots:
    void onClick();

protected:
    QVector<CollapseItem*> items;
};

#endif // COMPONENT_H
#endif
