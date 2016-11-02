#include <QString>
#include <QtTest>

class Graph : public QObject
{
    Q_OBJECT

public:
    Graph();

private Q_SLOTS:
    void addNode();
};

Graph::Graph()
{
}

void Graph::addNode()
{
    QVERIFY2(true, "Failure");
}

QTEST_APPLESS_MAIN(Graph)

#include "tst_graph.moc"
