#ifdef GUI
#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include "stdafx.h"
#include "settings.h"
#include "component.h"
#include "start_widget.h"
#include <QScrollArea>
//#include <QGroupBox>
//#include <QGridLayout>
//#include <QMap>
#include <QLineEdit>
#include <iostream>
#include <QListWidget>
#include <QSharedPointer>
#include <QRadioButton>

enum EditType {
    EDIT_NUMBER = 0,
    EDIT_INPUT  = 1,
    EDIT_TRACK  = 2,
    EDIT_RATIO  = 3,
    EDIT_SELECT = 4
};

typedef struct {
    int type;
    QString key;
    QVariant value;
}EditData;

class EditItem: public CollapseItem {
    Q_OBJECT
public:
    EditItem(const QString& name, QSize sz, int width_key, QWidget* parent=nullptr):
            CollapseItem(name, sz, parent), w_key(width_key) { }

    EditItem(const QString& name, int w, int h, int width_key, QWidget* parent=nullptr):
            CollapseItem(name, w, h, parent), w_key(width_key) { }

    void addInputItem(const QString& name, std::string* pvalue);

    void addTrackbarItem(const QString& name, int min, int max, int* pvalue);

private:

    QVector<EditData> datas;
    int w_key;

};

class TrackbarItemWidget: public QWidget {
    Q_OBJECT
public:
    TrackbarItemWidget(const QString& name, int w, int w_key, int min, int max, int* pvalue,
            int value_default, QWidget* parent=nullptr): QWidget(parent) {

        this->setFixedWidth(w);
        QHBoxLayout* l = new QHBoxLayout();

        QLabel* key = new QLabel(name, this);
        key->setFixedWidth(w_key);

        QSlider* slider = new QSlider(Qt::Horizontal, this);
        slider->setRange(min, max);
        slider->setValue(value_default);

        cnt = new QSpinBox(this);
        cnt->setRange(min, max);
        cnt->setFixedWidth(50);
        cnt->setValue(value_default);

        connect(slider, SIGNAL(valueChanged(int)), cnt, SLOT(setValue(int)));
        connect(cnt, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));

        l->addWidget(key);
        l->addWidget(slider);
        l->addWidget(cnt);
        setLayout(l);
        adjustSize();

        bind(pvalue);

        connect(cnt, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
    }

    void bind(int* integer) {
        i = integer;
    }

private slots:
    void onValueChanged(int v) {
        *i = v;
        Data::getData().flag_update_param = true;
    }

private:
    int* i;
    QSpinBox* cnt;
};

class InputItemWidget: public QWidget {
    Q_OBJECT
public:
    InputItemWidget(const QString& name, int w, int w_key, std::string* pvalue,
            const QString& value_default="", QWidget* parent=nullptr): QWidget(parent) {

        this->setFixedWidth(w);
        QHBoxLayout* l = new QHBoxLayout();

        QLabel* key = new QLabel(name, this);
        key->setFixedWidth(w_key);

        input = new QLineEdit(this);
        input->setText(value_default);

        l->addWidget(key);
        l->addWidget(input);
        this->setLayout(l);

        bind(pvalue);

        connect(input, SIGNAL(textChanged(const QString&)), this, SLOT(onTextChange(const QString&)));
    }

    void bind(std::string* s) {
        str = s;
    }

private slots:
    void onTextChange(const QString& txt) {
        *str = txt.toStdString();
    }

private:
    std::string* str;
    QLineEdit* input;
};


class MainWidget;
class ShowArea: public QWidget {
    Q_OBJECT
public:
    explicit ShowArea(MainWidget* parent=nullptr);

    void paintEvent(QPaintEvent *event) override;

    void display();

private slots:
    void updateContent(const QString& key, const QString& value, bool is_show_val);

private:
    QGridLayout* l;
    QVector<QLabel*> lbls;
    QMap<QString, QPair<int, QString>> map;
    int frame_update;
    int frame_cur;
};

class ControlArea: public QWidget {
    Q_OBJECT
public:
    explicit ControlArea(QWidget* parent=nullptr);

    void paintEvent(QPaintEvent *event) override;

signals:
    void changeMode(int);
    void save(bool);

private slots:
    void onModeChanged(int mode);

    void onSave(bool clicked) {
        Parameter::getTempParameter().other.flag_save = clicked;
        Data::getData().flag_update_param = true;
    }

    void onColorChanged(int color);

private:

    QButtonGroup* radios_mode;
    QButtonGroup* radios_color;
    QRadioButton* radio_armor;
    QRadioButton* radio_buff;
    QRadioButton* radio_red;
    QRadioButton* radio_blue;
    CheckableButton* btn_save;
};

class MainWidget: public QWidget {
    Q_OBJECT
public:
    static void log(const QString& str, int color);
    static void log(const char* str, int color) {
        MainWidget::log(QString(str), color);
    }
    static void log(const std::string& str, int color) {
        MainWidget::log(QString(str.c_str()), color);
    }

    static void log2(const std::string& key, const std::string& value, bool is_show_val) {
        if (MainWidget::pointer)
            MainWidget::pointer->_log2(Tool::toString(key), Tool::toString(value), is_show_val);
    }

    void _log2(const std::string& key, const std::string& value, bool is_show_val) {
        emit sendLog(key.c_str(), value.c_str(), is_show_val);
    }
public:
    explicit MainWidget(QWidget* parent=nullptr);

    void display();

signals:
    void sendLog(const QString&, const QString&, bool);

private:
    static MainWidget* pointer;

private:
    bool is_cap_mode;

    QLabel* img_show;

    QScrollArea* area_edit;
    CollapseBox* box_edit;
    QVector<EditItem*> items_edit;

    QListWidget* area_console;

    ShowArea* area_show;
    ControlArea* area_ctrl;

};

#endif // MAIN_WIDGET_H
#endif
