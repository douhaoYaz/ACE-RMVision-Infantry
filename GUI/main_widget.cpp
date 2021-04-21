#include "main_widget.h"
#include "ui_tool.h"
#ifdef GUI
#include <QDebug>

MainWidget* MainWidget::pointer = nullptr;

void MainWidget::log(const QString& str, int color) {
    if (MainWidget::pointer)
        pointer->area_console->addItem(str);
} 

MainWidget::MainWidget(QWidget* parent): QWidget(parent) {

    Parameter& param_tmp = Parameter::getTempParameter();

    if (MainWidget::pointer == nullptr)
        pointer = this;

    this->setFixedSize(915, 670);

    QGridLayout* l = new QGridLayout();

    img_show = new QLabel(this);
    img_show->setFixedSize(640, 480);
    l->addWidget(img_show, 1, 1, 1, 2);


    area_edit = new QScrollArea(this);
    area_edit->setFixedSize(270, 480);
    area_edit->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);

    area_show = new ShowArea(this);
    area_show->setFixedSize(300, 185);
    l->addWidget(area_show, 2, 1, 1, 1);

    area_console = new QListWidget(this);
    area_console->setFixedSize(335, 185);
    l->addWidget(area_console, 2, 2, 1, 1);

    box_edit = new CollapseBox(area_edit);

    EditItem* items_armor = new EditItem("Armor", 250, 40, 65, box_edit);
    items_armor->addTrackbarItem("th_color", 0, 255,
            (int*)Reflector<Parameter::ArmorParameter>::get(&(param_tmp.armor), "th_color"));
    items_armor->addTrackbarItem("th_gray", 0, 255, &param_tmp.armor.th_gray);
    items_armor->addTrackbarItem("cap_x", 0, 200, &param_tmp.armor.pt_offset_cap.x);
    items_armor->addTrackbarItem("cap_y", 0, 200, &param_tmp.armor.pt_offset_cap.y);
    box_edit->addItem(items_armor); 

    EditItem* items_buff = new EditItem("Buff", 250, 40, 65, box_edit);
    items_buff->addTrackbarItem("th_color", 0, 255, &param_tmp.buff.th_color);
    items_buff->addTrackbarItem("th_gray", 0, 255, &param_tmp.buff.th_gray);
    items_buff->addTrackbarItem("cap_x", 0, 200, &param_tmp.buff.pt_offset_cap.x);
    items_buff->addTrackbarItem("cap_y", 0, 200, &param_tmp.buff.pt_offset_cap.y);
    items_buff->addTrackbarItem("world_x", 0, 1000, &param_tmp.buff.x_pt_offset_world);
    items_buff->addTrackbarItem("follow_x", 0, 200, &param_tmp.buff.pt_offset_follow_delay.x);
    items_buff->addTrackbarItem("follow_y", 0, 200, &param_tmp.buff.pt_offset_follow_delay.y);
    box_edit->addItem(items_buff);

    EditItem* items_angle = new EditItem("Angle", 250, 40, 65, box_edit);
    items_angle->addTrackbarItem("gravity", 0, 10, &param_tmp.angle.gravity_offset);
    box_edit->addItem(items_angle);

    area_edit->setWidget(box_edit);
    l->addWidget(area_edit, 1, 3, 1, 1);

    area_ctrl = new ControlArea(this);
    l->addWidget(area_ctrl, 2, 3, 1, 1);

    l->setMargin(0);
    this->setLayout(l);

}

void MainWidget::display() {
    Data& data = Data::getData();
    if (data.img_show.empty() == false) {
        cv::Mat img;
        cv::cvtColor(data.img_show, img, cv::COLOR_BGR2RGB);
        QImage qimg(img.data, img.cols, img.rows, QImage::Format_RGB888);
        img_show->setPixmap(QPixmap::fromImage(qimg));
        img_show->show();
    }
    area_show->display();
}

ShowArea::ShowArea(MainWidget* parent): QWidget(parent), frame_update(2000), frame_cur(0) {
    l = new QGridLayout();
    this->setLayout(l);
    connect(parent, SIGNAL(sendLog(const QString&, const QString&, bool)),
            this, SLOT(updateContent(const QString&, const QString&, bool)));
}

void ShowArea::paintEvent(QPaintEvent*) {
    QStyleOption opt;
    opt.init(this);
    QPainter* painter = new QPainter(this);
    this->style()->drawPrimitive(QStyle::PE_Widget, &opt, painter, this);
    painter->end();
}

void ShowArea::updateContent(const QString& key, const QString& value, bool is_show_val) {

    auto iter = map.find(key);
    if (iter == map.end()) {
        int sz = lbls.size();
        map.insert(key, { lbls.size(), value });

        QLabel* lbl_val = new QLabel(value, this);
        if (is_show_val) {
            l->addWidget(new QLabel(key, this), sz, 0, Qt::AlignLeft);
            l->addWidget(lbl_val, sz, 1, Qt::AlignLeft);
        }
        else
            l->addWidget(lbl_val, sz, 0, 1, 2, Qt::AlignLeft);

        lbls.push_back(lbl_val);
    }
    else
        iter.value().second = value;

}

void ShowArea::display() {
    if (frame_cur++ >= frame_update)
        for (auto& v: map)
            lbls[v.first]->setText(v.second);

}

ControlArea::ControlArea(QWidget* parent): QWidget(parent) {
    QGridLayout* layout = new QGridLayout();

    layout->addWidget(new QLabel("mode:", this), 0, 0);

    radio_armor = new QRadioButton("armor", this);
    layout->addWidget(radio_armor, 0, 1);

    radio_buff = new QRadioButton("buff", this);
    layout->addWidget(radio_buff, 0, 2);

    radios_mode = new QButtonGroup();
    radios_mode->addButton(radio_armor, DETECT_ARMOR);
    radios_mode->addButton(radio_buff, DETECT_BUFF);
    radio_armor->setChecked(true);

    layout->addWidget(new QLabel("color:", this), 1, 0);

    radio_red = new QRadioButton("red", this);
    layout->addWidget(radio_red, 1, 1);

    radio_blue = new QRadioButton("blue", this);
    layout->addWidget(radio_blue, 1, 2);

    radios_color = new QButtonGroup();
    radios_color->addButton(radio_red, TARGET_RED);
    radios_color->addButton(radio_blue, TARGET_BLUE);
    if (Parameter::getParameter().other.color_enemy == TARGET_RED)
        radio_red->setChecked(true);
    else
        radio_blue->setChecked(true);

    btn_save = new CheckableButton("save", this);
    layout->addWidget(btn_save, 2, 0, 1, 3);
    this->setLayout(layout);

    connect(radios_mode, SIGNAL(buttonClicked(int)), this, SLOT(onModeChanged(int)));
    connect(radios_color, SIGNAL(buttonClicked(int)), this, SLOT(onColorChanged(int)));
    connect(btn_save, SIGNAL(clicked(bool)), this, SLOT(onSave(bool)));
}

void ControlArea::paintEvent(QPaintEvent*) {
    QStyleOption opt;
    opt.init(this);
    QPainter* painter = new QPainter(this);
    this->style()->drawPrimitive(QStyle::PE_Widget, &opt, painter, this);
    painter->end();
}

void ControlArea::onModeChanged(int mode) {
    if (mode == Parameter::getParameter().other.mode_detect)
        return;
    if (mode == DETECT_ARMOR)
        UITool::log("Change to ARMOR mode!");

    else if (mode == DETECT_BUFF)
        UITool::log("Change to BUFF mode!");

    else {
        UITool::log("Error mode!");
        return;
    }
    Parameter::getTempParameter().other.mode_detect = mode;
    Data::getData().flag_update_param = true;
}

void ControlArea::onColorChanged(int color) {
    if (color == Parameter::getParameter().other.color_enemy)
        return;
    if (color == TARGET_RED)
        UITool::log("Now color of enemy is RED!");
    else if (color == TARGET_BLUE)
        UITool::log("Now color of enemy is BLUE!");
    else {
        UITool::log("Error color!");
        return;
    }
    Parameter::getTempParameter().other.color_enemy = color;
    Data::getData().flag_update_param = true;
}

void EditItem::addInputItem(const QString& name, std::string* pvalue) {
    InputItemWidget* widget = new InputItemWidget(name, btn->size().width(),
            w_key, pvalue, QString(pvalue->c_str()), this);
    CollapseItem::addItem(widget);
}

void EditItem::addTrackbarItem(const QString& name, int min, int max, int* pvalue) {
    TrackbarItemWidget* widget = new TrackbarItemWidget(name, btn->size().width(),
            w_key, min, max, pvalue, *pvalue, this);
    CollapseItem::addItem(widget);
}

#endif
