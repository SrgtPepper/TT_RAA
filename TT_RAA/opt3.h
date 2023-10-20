#ifndef OPT3_H
#define OPT3_H

#include <QDialog>

namespace Ui {
class opt3;
}

class opt3 : public QDialog
{
    Q_OBJECT

public:
    explicit opt3(QWidget *parent = nullptr);
    ~opt3();

private:
    Ui::opt3 *ui;
};

#endif // OPT3_H
