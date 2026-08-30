#pragma once
#include <QDialog>
#include <string>
#include <vector>
#include <libudev.h>
#include "yaml_config.h"

QT_BEGIN_NAMESPACE
namespace Ui { class DialogSerialSetting;}
QT_END_NAMESPACE

class DlgSerialSetting : public QDialog
{
    Q_OBJECT

public:
    DlgSerialSetting(QWidget *parent = nullptr);
    ~DlgSerialSetting();
protected:
    void showEvent(QShowEvent *event) override;
    
private slots:
    void slot_btn_cancle_clicked();
    void slot_btn_apply_clicked();


    
public:
    void find_serial();
    void update_combobox();
    
signals:
    void sig_setserial_finished();

public:
    std::vector<std::string> serial_name;
    std::string config_path;
    std::string gnss_config_path;
private:
    Ui::DialogSerialSetting* ui;
    YamlConfig all_yaml_config;
    YamlConfig gnss_yaml_config;
    YAML::Node all_config;
    YAML::Node gnss_config;
};