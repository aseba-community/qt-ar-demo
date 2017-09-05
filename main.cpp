#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "thymio-ar/thymio-ar.h"

#include "markermodel.h"

int main(int argc, char* argv[]) {

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QGuiApplication app(argc, argv);

    app.setOrganizationName("Thymio");
	app.setOrganizationDomain("thymio.org");
	app.setApplicationName("Thymio AR demo");

    qmlRegisterType<MarkerModel>("MarkerModel", 1, 0, "MarkerModel");

	thymioARInit();

	QQmlApplicationEngine engine;
	engine.load(QUrl("qrc:/main.qml"));

	return app.exec();
}
