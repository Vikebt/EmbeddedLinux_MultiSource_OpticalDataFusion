#include "dlgImageMatchShow.h"
#include "ui_dlgImageMatchShow.h"
#include <QPixmap>

DlgImageMatchShow::DlgImageMatchShow(QWidget *parent)
:QDialog(parent),ui(new Ui::DialogImageMatchShow)
{
    ui->setupUi(this); 
}

DlgImageMatchShow::~DlgImageMatchShow()
{

}

void DlgImageMatchShow::slot_update_image(QImage img)
{
    QPixmap pixmap = QPixmap::fromImage(img);
    // ui->lable_image->setScaledContents(true);
    // ui->lable_image->setPixmap(pixmap.scaled(ui->label_display->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
    

    ui->label_image->setScaledContents(false);  // 必须设为false，否则手动缩放无效！
    ui->label_image->setPixmap(
        pixmap.scaled(
        ui->label_image->size(), 
        Qt::KeepAspectRatio,               // 关键：保持宽高比
        Qt::SmoothTransformation           // 平滑缩放
    )
);

    // // 延迟10毫秒执行缩放（确保布局完成）
    // QTimer::singleShot(10, this, [=]() {
    //     ui->label_image->setScaledContents(false);
    //     ui->label_image->setPixmap(
    //         pixmap.scaled(
    //             ui->label_image->size(),
    //             Qt::KeepAspectRatio,
    //             Qt::SmoothTransformation
    //         )
    //     );
    // });
}
