#ifndef OPT1_H
#define OPT1_H

#include <QDialog>

namespace Ui {
class opt1;
}

class opt1 : public QDialog
{
    Q_OBJECT

public:
    explicit opt1(QWidget *parent = nullptr);
    ~opt1();

private:
    Ui::opt1 *ui;
};

#endif // OPT1_H
