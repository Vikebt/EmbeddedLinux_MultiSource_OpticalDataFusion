#pragma once
#include <QDialog>
#include <QImage>
QT_BEGIN_NAMESPACE
namespace Ui { class DialogImageMatchShow;}
QT_END_NAMESPACE

class DlgImageMatchShow : public QDialog
{
    Q_OBJECT

public:
    DlgImageMatchShow(QWidget *parent = nullptr);
    ~DlgImageMatchShow();
// protected:
//     void showEvent(QShowEvent *event) override;
    
    void slot_update_image(QImage img);

private:
    Ui::DialogImageMatchShow* ui;
  
};