#ifndef OPT2_H
#define OPT2_H

#include <QDialog>

namespace Ui {
class opt2;
}

class opt2 : public QDialog
{
    Q_OBJECT

public:
    explicit opt2(QWidget *parent = nullptr);
    ~opt2();

private:
    Ui::opt2 *ui;
};

#endif // OPT2_H
