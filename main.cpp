#include "collection.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	
	QFont font;
	font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
	font.setPointSize(18);

    Collection w;
	w.setFont(font);
    w.show();
    return a.exec();
}
