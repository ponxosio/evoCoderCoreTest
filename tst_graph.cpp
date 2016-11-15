#include <stdexcept>

//Qt
#include <QString>
#include <QtTest>
#include <QTemporaryDir>
#include <QCryptographicHash>

//LIB
#include <easylogging++.h>

//LOCAL
#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/Edge.h>
#include <graph/Flow.h>

#include <util/AutoEnumerate.h>
#include <util/logutils.h>

//executable

#include <fluidControl/machineGraph/MachineGraph.h>
#include <fluidControl/executable/ExecutableMachineGraph.h>
#include <fluidControl/executable/containers/actuators/communications/CommandSender.h>

#include <fluidControl/ExecutionEngine.h>
#include <fluidControl/mapping/MappingEngine.h>
#include <fluidControl/machineGraph/MachineGraph.h>
#include <fluidControl/machineGraph/ContainerNode.h>
#include <fluidControl/machineGraph/ContainerNodeType.h>

#include <fluidControl/executable/containers/actuators/liquids/Control.h>
#include <fluidControl/executable/containers/actuators/liquids/Injector.h>
#include <fluidControl/executable/containers/actuators/liquids/Extractor.h>

#include <fluidControl/executable/containers/actuators/extras/Temperature.h>
#include <fluidControl/executable/containers/actuators/extras/ODSensor.h>
#include <fluidControl/executable/containers/actuators/extras/Light.h>
#include <fluidControl/executable/containers/actuators/extras/Mixer.h>

#include <fluidControl/executable/containers/actuators/communications/CommunicationsInterface.h>
#include <fluidControl/executable/containers/actuators/communications/CommandSender.h>
#include <fluidControl/executable/containers/actuators/communications/FileSender.h>
#include <fluidControl/executable/containers/actuators/communications/SerialSender.h>

#include <fluidControl/executable/containers/InletContainer.h>
#include <fluidControl/executable/containers/BidirectionalSwitch.h>
#include <fluidControl/executable/containers/ConvergentSwitch.h>
#include <fluidControl/executable/containers/ConvergentSwitchInlet.h>
#include <fluidControl/executable/containers/DivergentSwitch.h>
#include <fluidControl/executable/containers/DivergentSwitchSink.h>
#include <fluidControl/executable/containers/ExecutableContainerNode.h>
#include <fluidControl/executable/containers/FlowContainer.h>
#include <fluidControl/executable/containers/SinkContainer.h>

//operations
#include <protocolGraph/ConditionEdge.h>
#include <protocolGraph/operations/AssignationOperation.h>
#include <protocolGraph/operations/container/ApplyLight.h>
#include <protocolGraph/operations/container/ApplyTemperature.h>
#include <protocolGraph/operations/container/ContainerOperation.h>
#include <protocolGraph/operations/container/GetVolume.h>
#include <protocolGraph/operations/container/LoadContainerOperation.h>
#include <protocolGraph/operations/container/MeasureOD.h>
#include <protocolGraph/operations/container/Mix.h>
#include <protocolGraph/operations/container/SetContinousFlow.h>
#include <protocolGraph/operations/container/TimeStep.h>
#include <protocolGraph/operations/container/Transfer.h>
#include <protocolGraph/operations/DivergeNode.h>
#include <protocolGraph/operations/LoopNode.h>
#include <protocolGraph/ProtocolGraph.h>

//operables

#include <operables/mathematics/ConstantNumber.h>
#include <operables/mathematics/ArithmeticOperation.h>
#include <operables/mathematics/UnaryOperation.h>
#include <operables/mathematics/MathematicOperable.h>
#include <operables/mathematics/VariableEntry.h>

#include <operables/VariableTable.h>

#include <operables/comparison/ComparisonOperable.h>
#include <operables/comparison/Tautology.h>
#include <operables/comparison/SimpleComparison.h>
#include <operables/comparison/BooleanComparison.h>

//plugins
#include <plugin/PluginFileLoader.h>
#include <plugin/PythonEnvironment.h>
#include <plugin/actuators/ODSensorPlugin.h>
#include <plugin/actuators/MixerPlugin.h>
#include <plugin/actuators/TemperaturePlugin.h>
#include <plugin/actuators/LightPlugin.h>
#include <plugin/actuators/ControlPlugin.h>
#include <plugin/actuators/ExtractorPlugin.h>
#include <plugin/actuators/InjectorPlugin.h>

#include <ExecutionServer.h>
#include <ExecutionMachineServer.h>
#include <BioBlocksJSONReader.h>

INITIALIZE_NULL_EASYLOGGINGPP

class GraphTest : public QObject
{
    Q_OBJECT

public:
    GraphTest();

private Q_SLOTS:
    void initTestCase();
    void init();

    void graphConstructionAdd();
    void graphConstructionRemove();

    void testPluginFileLoader();

    void testProtocolGraph();
    void testMachineGraph();
    void testExecutableMachineGraph();

    void testPathManager();
    void testMappingEngineDone();
    void testMappingEngineFails();

    void testExecutionEngine();
    void testParseVolume();
    void testParseTime();
    void testParseFlow();
    void testBioBlocksJSONReader();

    void cleanupTestCase();
    void cleanup();

private:
     std::shared_ptr<Graph<Node, Edge>> makeGraph();
     MachineGraph* makeTurbidostatSketch();
     MachineGraph* makeComplexSketch();
     MachineGraph* makeImposibleSketch();
     ProtocolGraph* makeTimeProtocol();
     ExecutableMachineGraph* makeMappingMachine(int communications,
                                                std::unique_ptr<CommandSender> exec,
                                                std::unique_ptr<CommandSender> test);

};

GraphTest::GraphTest()
{

}

void GraphTest::initTestCase() {
    el::Helpers::setStorage(sharedLoggingRepository());

    el::Configurations c("log.ini");
    el::Loggers::reconfigureAllLoggers(c);

    PythonEnvironment::GetInstance()->initEnvironment();
}

void GraphTest::init() {
    //PythonEnvironment::GetInstance()->initEnvironment();
}

void GraphTest::cleanupTestCase() {
    PythonEnvironment::GetInstance()->finishEnvironment();
}

void GraphTest::cleanup() {
    //PythonEnvironment::GetInstance()->finishEnvironment();
}

void GraphTest::graphConstructionAdd()
{
    std::shared_ptr<Graph<Node, Edge>> g = std::make_shared<Graph<Node, Edge>>();

    QVERIFY2(g->addNode(std::make_shared<Node>(0)), "Failure ading node 0");
    QVERIFY2(g->addNode(std::make_shared<Node>(1)), "Failure ading node 1");
    QVERIFY2(g->addNode(std::make_shared<Node>(2)), "Failure ading node 2");
    QVERIFY2(g->addNode(std::make_shared<Node>(3)), "Failure ading node 3");

    QVERIFY2(g->addEdge(std::make_shared<Edge>(0, 1)), "Failure connecting nodes 0, 1");
    QVERIFY2(g->addEdge(std::make_shared<Edge>(1, 2)), "Failure connecting nodes 1, 2");
    QVERIFY2(g->addEdge(std::make_shared<Edge>(3, 1)), "Failure connecting nodes 3, 1");

    const Graph<Node, Edge>::EdgeVectorPtr arriving = g->getArrivingEdges(1);
    QVERIFY2(arriving->size() == 2, "Failure arriving nodes to 1 are not 2");
    QVERIFY2(arriving->at(0)->getIdSource() == 0  &&
             arriving->at(0)->getIdTarget() == 1 &&
             arriving->at(1)->getIdSource() == 3 &&
             arriving->at(1)->getIdTarget() == 1, "Failure arriving nodes are not the ones inserted");

    const Graph<Node, Edge>::EdgeVectorPtr leaving = g->getLeavingEdges(1);
    QVERIFY2(leaving->size() == 1, "Failure arriving nodes to 1 are not 2");
    QVERIFY2(leaving->at(0)->getIdSource() == 1 &&
             leaving->at(0)->getIdTarget() == 2, "Failure leaving nodes are not the ones inserted");

}

void GraphTest::graphConstructionRemove() {
    std::shared_ptr<Graph<Node, Edge>> g = makeGraph();

    size_t initialSize = g->getEdgeList()->size();
    g->removeEdge(Edge(3,1));
    QVERIFY2(initialSize - g->getEdgeList()->size() == 1, "Fail edge 3->1 not deleted");
    initialSize -= 1;

    QVERIFY2(g->removeNode(1), "Fail removing node 1");
    QVERIFY2(initialSize - g->getEdgeList()->size() == 2, "Fail not all edges connected to 1 are deleted");
    QVERIFY2(g->getArrivingEdges(1) == NULL, "Arriving edges of node 1 are not deleted");
    QVERIFY2(g->getLeavingEdges(1) == NULL, "Leaving edges of node 1 are not deleted");
}

std::shared_ptr<Graph<Node, Edge>> GraphTest::makeGraph() {
    std::shared_ptr<Graph<Node, Edge>> g = std::make_shared<Graph<Node, Edge>>();

    g->addNode(std::make_shared<Node>(0));
    g->addNode(std::make_shared<Node>(1));
    g->addNode(std::make_shared<Node>(2));
    g->addNode(std::make_shared<Node>(3)) ;
    g->addEdge(std::make_shared<Edge>(0, 1));
    g->addEdge(std::make_shared<Edge>(1, 2));
    g->addEdge(std::make_shared<Edge>(3, 1)) ;

    return g;
}

void GraphTest::testPluginFileLoader() {
    //PluginFileLoader::setPluginDir(":/resources/pythonPlugins");
    PluginFileLoader* loader = PluginFileLoader::GetInstance();

    std::vector<std::string> spectedNames = {"Control",
                                             "Extractor",
                                             "Injector",
                                             "Light",
                                             "Mixer",
                                             "OdSensor",
                                             "Temperature",
                                             "EvoprogDummyInjector",
                                             "EvoprogOdSensor",
                                             "Evoprog4WayValve",
                                             "EvoprogV2Pump"};

    std::vector<std::string> allNames = loader->getAllNames();
    for (std::string spected: spectedNames) {
        bool finded = std::find(allNames.begin(), allNames.end(), spected) != allNames.end();
        QVERIFY2(finded, QString::fromStdString(spected + " not found").toUtf8().constData());
    }
}

void GraphTest::testProtocolGraph() {
    std::shared_ptr<ProtocolGraph> protocol(makeTimeProtocol());

    QTemporaryDir temp;
    if (temp.isValid()) {
        QString pathTempProtocol = temp.path() + "/timeProtocol.graph";
        QString pathTempProtocolJSON = temp.path() + "/timeProtocol.json";

        protocol->printProtocol(pathTempProtocol.toUtf8().constData());
        ProtocolGraph::toJSON(pathTempProtocolJSON.toUtf8().constData(), *protocol.get());

        QByteArray sha1Protocol;
        QCryptographicHash hashProtocol(QCryptographicHash::Sha1);

        QFile fileProtocol(pathTempProtocol);
        if (fileProtocol.open(QFile::ReadOnly)) {
            if (hashProtocol.addData(&fileProtocol)) {
                sha1Protocol = hashProtocol.result();
            }
        }

        hashProtocol.reset();

        QByteArray sha1ProtocolJSON;
        QFile jsonProtocol(pathTempProtocolJSON);
        if (jsonProtocol.open(QFile::ReadOnly)) {
            if (hashProtocol.addData(&jsonProtocol)) {
                sha1ProtocolJSON = hashProtocol.result();
            }
        }

        QByteArray sha1Resource;
        QCryptographicHash hashResource(QCryptographicHash::Sha1);
        QFile fileResource(":/resources/timeProtocol.graph");
        if (fileResource.open(QFile::ReadOnly)) {
            if (hashResource.addData(&fileResource)) {
                sha1Resource = hashResource.result();
            }
        }

        hashResource.reset();

        QByteArray sha1ResourceJSON;
        QFile resourceJSON(":/resources/timeProtocol.json");
        if (resourceJSON.open(QFile::ReadOnly)) {
            if (hashResource.addData(&resourceJSON)) {
                sha1ResourceJSON = hashResource.result();
            }
        }

        QVERIFY2(sha1Protocol.size() != 0, "SHA1 value for protocol graph generated by the test is empty");
        QVERIFY2(sha1Resource.size() != 0, "SHA1 value for resource graph file is empty");
        QVERIFY2(sha1Protocol == sha1Resource, "Generated graph files are not the same");

        QVERIFY2(sha1ProtocolJSON.size() != 0, "SHA1 value for protocol JSON generated by the test is empty");
        QVERIFY2(sha1ResourceJSON.size() != 0, "SHA1 value for resource JSON file is empty");
        QVERIFY2(sha1ProtocolJSON == sha1ResourceJSON, "Generated JSON files are not the same");
    } else {
        QFAIL("Error of the Qt platform while creating temporary dir");
    }
}

void GraphTest::testMachineGraph() {
    std::shared_ptr<MachineGraph> sketch(makeTurbidostatSketch());

    QTemporaryDir temp;
    if (temp.isValid()) {
        QString pathTempSketch = temp.path() + "/sketch.graph";
        QString pathTempSketchJSON = temp.path() + "/sketch.json";

        sketch->printMachine(pathTempSketch.toUtf8().constData());
        MachineGraph::toJSON(pathTempSketchJSON.toUtf8().constData(), *sketch.get());

        QByteArray sha1Sketch;
        QCryptographicHash hashSketch(QCryptographicHash::Sha1);
        QFile fileSketch(pathTempSketch);
        if (fileSketch.open(QFile::ReadOnly)) {
            if (hashSketch.addData(&fileSketch)) {
                sha1Sketch = hashSketch.result();
            }
        }

        hashSketch.reset();

        QByteArray sha1SketchJSON;
        QFile fileSketchJSON(pathTempSketchJSON);
        if (fileSketchJSON.open(QFile::ReadOnly)) {
            if (hashSketch.addData(&fileSketchJSON)) {
                sha1SketchJSON = hashSketch.result();
            }
        }

        QByteArray sha1Resource;
        QCryptographicHash hashResource(QCryptographicHash::Sha1);
        QFile fileResource(":/resources/machineGraph.graph");
        if (fileResource.open(QFile::ReadOnly)) {
            if (hashResource.addData(&fileResource)) {
                sha1Resource = hashResource.result();
            }
        }

        hashResource.reset();

        QByteArray sha1ResourceJSON;
        QFile fileResourceJSON(":/resources/machineGraph.json");
        if (fileResourceJSON.open(QFile::ReadOnly)) {
            if (hashResource.addData(&fileResourceJSON)) {
                sha1ResourceJSON = hashResource.result();
            }
        }

        QVERIFY2(sha1Sketch.size() != 0, "SHA1 value for machine graph generated by the test is empty");
        QVERIFY2(sha1Resource.size() != 0, "SHA1 value for resource file is empty");
        QVERIFY2(sha1Sketch == sha1Resource, "Generated files are not the same");

        QVERIFY2(sha1SketchJSON.size() != 0, "SHA1 value for machine JSON file generated by the test is empty");
        QVERIFY2(sha1ResourceJSON.size() != 0, "SHA1 value for resource JSON file is empty");
        QVERIFY2(sha1SketchJSON == sha1ResourceJSON, "Generated JSON files are not the same");
    } else {
        QFAIL("Error of the Qt platform while creating temporary dir");
    }
}

void GraphTest::testExecutableMachineGraph() {
    std::unique_ptr<CommandSender> comEx = std::unique_ptr<CommandSender>(new SerialSender("\\\\.\\COM3"));
    std::unique_ptr<CommandSender> comTest = std::unique_ptr<CommandSender>(new FileSender("test.log", "inputFileData.txt"));
    int com = CommunicationsInterface::GetInstance()->addCommandSender(comEx->clone());

    std::shared_ptr<ExecutableMachineGraph> machine(makeMappingMachine(com, std::move(comEx), std::move(comTest)));

    QTemporaryDir temp;
    if (temp.isValid()) {
        QString pathTempSketchJSON = temp.path() + "/exMachine.json";
        ExecutableMachineGraph::toJSON(pathTempSketchJSON.toUtf8().constData(), *machine.get());

        QCryptographicHash hash(QCryptographicHash::Sha1);
        QByteArray sha1SketchJSON;
        QFile fileSketchJSON(pathTempSketchJSON);
        if (fileSketchJSON.open(QFile::ReadOnly)) {
            if (hash.addData(&fileSketchJSON)) {
                sha1SketchJSON = hash.result();
            }
        }

        hash.reset();

        QByteArray sha1ResourceJSON;
        QFile fileResourceJSON(":/resources/exMachine.json");
        if (fileResourceJSON.open(QFile::ReadOnly)) {
            if (hash.addData(&fileResourceJSON)) {
                sha1ResourceJSON = hash.result();
            }
        }

        QVERIFY2(sha1SketchJSON.size() != 0, "SHA1 value for machine JSON file generated by the test is empty");
        QVERIFY2(sha1ResourceJSON.size() != 0, "SHA1 value for resource JSON file is empty");
        QVERIFY2(sha1SketchJSON == sha1ResourceJSON, "Generated JSON files are not the same");
    } else {
        QFAIL("Error of the Qt platform while creating temporary dir");
    }
}

void GraphTest::testPathManager() {
    try {
        std::unique_ptr<CommandSender> comEx(new SerialSender("\\\\.\\COM3"));
        std::unique_ptr<CommandSender> comTest(new FileSender("test.log", "inputFileData.txt"));
        int communications = CommunicationsInterface::GetInstance()->addCommandSender(comEx->clone());

        std::shared_ptr<ExecutableMachineGraph> machine = std::shared_ptr<ExecutableMachineGraph>(makeMappingMachine(communications, std::move(comEx), std::move(comTest)));
        PathManager manager(machine);

        //2 to 7...

        std::vector<std::string> calculated27;
        shared_ptr<SearcherIterator> it = manager.getFlows(2, 7);
        while (it->hasNext()) {
            calculated27.push_back(it->next()->toText());
        }

        std::vector<std::string> spected27s = {"2->7:2->5;5->7;",
                                               "2->7:2->3;3->6;6->7;"};
        QVERIFY2(calculated27.size() == spected27s.size(), std::string(" path found size 2-7 are incongruent, expected: " +
                                                                       patch::to_string(spected27s.size()) +
                                                                       ", calculated: " +
                                                                       patch::to_string(calculated27.size())).c_str());

        for (std::string spected: spected27s) {
            bool finded = std::find(calculated27.begin(), calculated27.end(), spected) != calculated27.end();
            QVERIFY2(finded, std::string(spected + std::string(", not found in 2-7 paths")).c_str());
        }

        //2 to sink...

        std::vector<std::string> calculated2sink;
        it = manager.getFlows(2, make_shared<ContainerNodeType>(MovementType::irrelevant, ContainerType::sink));
        while (it->hasNext()) {
            calculated2sink.push_back(it->next()->toText());
        }

        std::vector<std::string> spected2sinks = {"2->3:2->3;",
                                                  "2->5:2->5;",
                                                  "2->6:2->3;3->6;",
                                                  "2->7:2->5;5->7;",
                                                  "2->7:2->3;3->6;6->7;"};
        QVERIFY2(calculated2sink.size() == spected2sinks.size(), std::string(" path found size 2-sink are incongruent, expected: " +
                                                                             patch::to_string(spected2sinks.size()) +
                                                                             ", calculated: " +
                                                                             patch::to_string(calculated2sink.size())).c_str());

        for (std::string spected: spected2sinks) {
            bool finded = std::find(calculated2sink.begin(), calculated2sink.end(), spected) != calculated2sink.end();
            QVERIFY2(finded, std::string(spected + std::string(", not found in 2-Sink paths")).c_str());
        }

        //inlet to 7...

        std::vector<std::string> calculatedInlet7;
        it = it = manager.getFlows(make_shared<ContainerNodeType>(MovementType::continuous, ContainerType::inlet), 7);
        while (it->hasNext()) {
            calculatedInlet7.push_back(it->next()->toText());
        }

        std::vector<std::string> spectedInlet7 = {"5->7:5->7;",
                                                  "6->7:6->7;",
                                                  "2->7:2->5;5->7;",
                                                  "2->7:2->3;3->6;6->7;",
                                                  "1->7:1->5;5->7;",
                                                  "4->7:4->6;6->7;",
                                                  "3->7:3->6;6->7;",
                                                  "6->7:6->2;2->5;5->7;",
                                                  "3->7:3->6;6->2;2->5;5->7;",
                                                  "4->7:4->6;6->2;2->5;5->7;"};
        QVERIFY2(calculatedInlet7.size() == spectedInlet7.size(), std::string(" path found size inlet-7 are incongruent, expected: " +
                                                                              patch::to_string(spectedInlet7.size()) +
                                                                              ", calculated: " +
                                                                              patch::to_string(calculatedInlet7.size())).c_str());

        for (std::string spected: spectedInlet7) {
            bool finded = std::find(calculatedInlet7.begin(), calculatedInlet7.end(), spected) != calculatedInlet7.end();
            QVERIFY2(finded, std::string(spected + std::string(", not found in inlet-7 paths")).c_str());
        }

        // flows from inlet to sink..

        std::vector<std::string> calculatedInletSink;
        it = manager.getFlows(make_shared<ContainerNodeType>(MovementType::continuous, ContainerType::inlet),
                              make_shared<ContainerNodeType>(MovementType::irrelevant, ContainerType::sink));
        while (it->hasNext()) {
            calculatedInletSink.push_back(it->next()->toText());
        }

        std::vector<std::string> spectedInletSink = {"1->5:1->5;",
                                                     "1->7:1->5;5->7;",
                                                     "5->7:5->7;",
                                                     "2->5:2->5;",
                                                     "2->7:2->5;5->7;",
                                                     "2->3:2->3;",
                                                     "2->6:2->3;3->6;",
                                                     "2->7:2->3;3->6;6->7;",
                                                     "3->6:3->6;",
                                                     "3->7:3->6;6->7;",
                                                     "3->2:3->6;6->2;",
                                                     "3->5:3->6;6->2;2->5;",
                                                     "3->7:3->6;6->2;2->5;5->7;",
                                                     "6->2:6->2;",
                                                     "6->5:6->2;2->5;",
                                                     "6->3:6->2;2->3;",
                                                     "6->7:6->7;",
                                                     "6->7:6->2;2->5;5->7;",
                                                     "4->6:4->6;",
                                                     "4->2:4->6;6->2;",
                                                     "4->7:4->6;6->7;",
                                                     "4->3:4->6;6->2;2->3;",
                                                     "4->5:4->6;6->2;2->5;",
                                                     "4->7:4->6;6->2;2->5;5->7;"};

        QVERIFY2(calculatedInletSink.size() == spectedInletSink.size(), std::string(" path found size inlet-sink are incongruent, expected: " +
                                                                                    patch::to_string(spectedInletSink.size()) +
                                                                                    ", calculated: " +
                                                                                    patch::to_string(calculatedInletSink.size())).c_str());

        for (std::string spected: spectedInletSink) {
            bool finded = std::find(calculatedInletSink.begin(), calculatedInletSink.end(), spected) != calculatedInletSink.end();
            QVERIFY2(finded, std::string(spected + std::string(", not found in inlet-sink paths")).c_str());
        }
    } catch (exception & e) {
        QFAIL(std::string("exception thrown, " + std::string(e.what())).c_str());
    }
}

void GraphTest::testMappingEngineDone() {
    try {
        std::unique_ptr<CommandSender> comEx = std::unique_ptr<CommandSender>(new SerialSender("\\\\.\\COM3"));
        std::unique_ptr<CommandSender> comTest = std::unique_ptr<CommandSender>(new FileSender("test.log", "inputFileData.txt"));
        int com = CommunicationsInterface::GetInstance()->addCommandSender(comEx->clone());

        MachineGraph* sketch = makeTurbidostatSketch();
        std::shared_ptr<ExecutableMachineGraph> machine(makeMappingMachine(com, std::move(comEx), std::move(comTest)));
        MappingEngine* map = new MappingEngine(sketch, machine);

        QVERIFY2(map->startMapping(), "mapping turbidostat cannot be done");

        std::unordered_set<int> usedNodes;
        std::unordered_set<std::string> usedEdges;

        std::vector<std::shared_ptr<ContainerNode>> nodes(*sketch->getGraph()->getAllNodes().get());
        for (std::shared_ptr<ContainerNode> node: nodes) {
            int mappedNode = map->getMappedContainerId(node->getContainerId());
            if (usedNodes.find(mappedNode) == usedNodes.end()) {
                usedNodes.insert(mappedNode);
            } else {
                QFAIL(std::string("turbidostat: node " + patch::to_string(node->getContainerId()) +
                                  "is mapped to a execution node that mapped before, " + patch::to_string(mappedNode)).c_str());
            }
        }

        std::vector<std::shared_ptr<Edge>> edges(*sketch->getGraph()->getEdgeList().get());
        for (std::shared_ptr<Edge> edge: edges) {
            Flow<Edge>* flow = map->getMappedEdge(edge);
            if (usedEdges.find(flow->toText()) == usedEdges.end()) {
                usedEdges.insert(flow->toText());
            } else {
                QFAIL(std::string("turbidostat: edge " + edge->toText() + " is mapped to an used edge " + flow->toText()).c_str());
            }
        }

        delete map;
        delete sketch;
        usedNodes.clear();
        usedEdges.clear();

        sketch = makeComplexSketch();

        std::unique_ptr<CommandSender> comEx2 = std::unique_ptr<CommandSender>(new SerialSender("\\\\.\\COM3"));
        std::unique_ptr<CommandSender> comTest2 = std::unique_ptr<CommandSender>(new FileSender("test.log", "inputFileData.txt"));
        int com2 = CommunicationsInterface::GetInstance()->addCommandSender(comEx2->clone());
        std::shared_ptr<ExecutableMachineGraph> machine2(makeMappingMachine(com2, std::move(comEx2), std::move(comTest2)));

        map = new MappingEngine(sketch, machine2);

        QVERIFY2(map->startMapping(), "mapping complex cannot be done");

        std::vector<std::shared_ptr<ContainerNode>> nodesC(*sketch->getGraph()->getAllNodes().get());
        for (std::shared_ptr<ContainerNode> node: nodesC) {
            int mappedNode = map->getMappedContainerId(node->getContainerId());
            if (usedNodes.find(mappedNode) == usedNodes.end()) {
                usedNodes.insert(mappedNode);
            } else {
                QFAIL(std::string("complex: node " + patch::to_string(node->getContainerId()) +
                                  "is mapped to a execution node that mapped before, " + patch::to_string(mappedNode)).c_str());
            }
        }

        std::vector<std::shared_ptr<Edge>> edgesC(*sketch->getGraph()->getEdgeList().get());
        for (std::shared_ptr<Edge> edge: edgesC) {
            Flow<Edge>* flow = map->getMappedEdge(edge);
            if (usedEdges.find(flow->toText()) == usedEdges.end()) {
                usedEdges.insert(flow->toText());
            } else {
                QFAIL(std::string("complex: edge " + edge->toText() + " is mapped to an used edge " + flow->toText()).c_str());
            }
        }

        delete map;
        delete sketch;
    } catch (exception & e) {
        QFAIL(std::string("exception occured" + std::string(e.what())).c_str());
    }
}

void GraphTest::testMappingEngineFails() {
    try{
        std::unique_ptr<CommandSender> comEx = std::unique_ptr<CommandSender>(new SerialSender("\\\\.\\COM3"));
        std::unique_ptr<CommandSender> comTest = std::unique_ptr<CommandSender>(new FileSender("test.log", "inputFileData.txt"));
        int com = CommunicationsInterface::GetInstance()->addCommandSender(comEx->clone());

        MachineGraph* sketch = makeImposibleSketch();
        std::shared_ptr<ExecutableMachineGraph> machine(makeMappingMachine(com, std::move(comEx), std::move(comTest)));
        MappingEngine* map = new MappingEngine(sketch, machine);

        QVERIFY2(!map->startMapping(), "mapping imposible have being done when should not");
    } catch (exception & e) {
        QFAIL(std::string("exception while executing test, " + std::string(e.what())).c_str());
    }
}

void GraphTest::testExecutionEngine() {
    try {
        ExecutionServer* server = ExecutionServer::GetInstance();
        string machineRef = ExecutionMachineServer::GetInstance()->addNewMachine("exMachine.json");

        string ref1 = server->addProtocolOnNewMachine("timeProtocol.json", "exMachine.json");
        string ref2 = server->addProtocolOnExistingMachine("timeProtocol.json", machineRef);

        server->test(ref1);
        server->test(ref2);

        auto vec = ExecutionMachineServer::GetInstance()->getMachineMap();
        QVERIFY2(vec->size() == 2, "There are not 2 machines");
    } catch (exception & e) {
        QFAIL(std::string("exeception while executing, " + std::string(e.what())).c_str());
    }
}

void GraphTest::testBioBlocksJSONReader() {
    try {
        BioBlocksJSONReader reader("bioBlocksProtocol.json", 1000);

        std::shared_ptr<ProtocolGraph> translated = reader.getProtocol();
        translated->printProtocol("protocolTranslated.graph");

        QTemporaryDir temp;
        if (temp.isValid()) {
            QString pathTranslatedJSON = temp.path() + "/translatedProtocol.json";
            ProtocolGraph::toJSON(pathTranslatedJSON.toUtf8().constData(), *translated.get());
            ProtocolGraph::toJSON("bioBlocksTranslated.json", *translated.get());

            QCryptographicHash hash(QCryptographicHash::Sha1);
            QByteArray sha1TranslatedJSON;
            QFile fileTranslatedJSON(pathTranslatedJSON);
            if (fileTranslatedJSON.open(QFile::ReadOnly)) {
                if (hash.addData(&fileTranslatedJSON)) {
                    sha1TranslatedJSON = hash.result();
                }
            }

            hash.reset();

            QByteArray sha1ResourceJSON;
            QFile fileResourceJSON(":/resources/bioBlocksTranslated.json");
            if (fileResourceJSON.open(QFile::ReadOnly)) {
                if (hash.addData(&fileResourceJSON)) {
                    sha1ResourceJSON = hash.result();
                }
            }

            QVERIFY2(sha1TranslatedJSON.size() != 0, "SHA1 value for bioblock trasnlated JSON file generated by the test is empty");
            QVERIFY2(sha1ResourceJSON.size() != 0, "SHA1 value for resource bioblocks JSON file is empty");
            QVERIFY2(sha1TranslatedJSON == sha1ResourceJSON, "Generated JSON files are not the same");
        } else {
            QFAIL("Error of the Qt platform while creating temporary dir");
        }
    } catch (std::exception & e) {
        QFAIL(std::string("exeception while executing, " + std::string(e.what())).c_str());
    }
}

void GraphTest::testParseVolume() {
    BioBlocksJSONReader reader("bioBlocksProtocol.json", 1000);
    std::string volumeStr = "5:milliliter";
    double volumeValue = reader.parseVolume(volumeStr);
    QVERIFY2(volumeValue == 5, std::string("time value : " + patch::to_string(volumeValue)).c_str());
}

void GraphTest::testParseTime() {
    BioBlocksJSONReader reader("bioBlocksProtocol.json", 1000);
    std::string timeStr = "5:hours";
    double timeValue = reader.parseTime(timeStr);
    QVERIFY2(timeValue == 1.8e+7, std::string("time value : " + patch::to_string(timeValue)).c_str());
}

void GraphTest::testParseFlow() {
    BioBlocksJSONReader reader("bioBlocksProtocol.json", 1000);
    std::string flowStr = "900:milliliter/hours";
    double flowValue = reader.parseFlowRate(flowStr);
    QVERIFY2(flowValue == (900/3.6e+6), std::string("flow value : " + patch::to_string(flowValue)).c_str());
}

MachineGraph* GraphTest::makeTurbidostatSketch() {
    MachineGraph* sketch = new MachineGraph("sketchTurbidostat");

    std::shared_ptr<ContainerNodeType> cinlet(new ContainerNodeType(MovementType::continuous, ContainerType::inlet));
    std::shared_ptr<ContainerNodeType> cFlow(new ContainerNodeType(MovementType::continuous, ContainerType::flow));
    std::shared_ptr<ContainerNodeType> sink(new ContainerNodeType(MovementType::irrelevant, ContainerType::sink));

    sketch->addContainer(1, cinlet, 100.0);
    sketch->addContainer(2, cFlow, 100.0);
    sketch->addContainer(3, sink, 100.0);

    sketch->connectContainer(1, 2);
    sketch->connectContainer(2, 3);

    return sketch;
}

MachineGraph* GraphTest::makeComplexSketch() {
    MachineGraph* sketch = new MachineGraph("sketchTurbidostat");

    std::shared_ptr<ContainerNodeType> cinlet(new ContainerNodeType(MovementType::continuous, ContainerType::inlet));
    std::shared_ptr<ContainerNodeType> cFlow(new ContainerNodeType(MovementType::continuous, ContainerType::flow));
    std::shared_ptr<ContainerNodeType> convergentSwitch(new ContainerNodeType(MovementType::irrelevant, ContainerType::convergent_switch));
    std::shared_ptr<ContainerNodeType> bidirectionalT(new ContainerNodeType(MovementType::continuous, ContainerType::bidirectional_switch));
    std::shared_ptr<ContainerNodeType> cnvSwitchInlet(new ContainerNodeType(MovementType::continuous, ContainerType::convergent_switch_inlet));
    std::shared_ptr<ContainerNodeType> dvrSwitchSink(new ContainerNodeType(MovementType::continuous, ContainerType::divergent_switch_sink));

    sketch->addContainer(1, cinlet, 100.0);
    sketch->addContainer(2, dvrSwitchSink, 100.0);
    sketch->addContainer(3, cFlow, 100.0);
    sketch->addContainer(4, cinlet, 100.0);
    sketch->addContainer(5, cnvSwitchInlet, 100.0);
    sketch->addContainer(6, bidirectionalT, 100.0);
    sketch->addContainer(7, convergentSwitch, 100.0);

    sketch->connectContainer(1, 5);
    sketch->connectContainer(5, 7);
    sketch->connectContainer(2, 5);
    sketch->connectContainer(2, 3);
    sketch->connectContainer(6, 2);
    sketch->connectContainer(6, 7);
    sketch->connectContainer(3, 6);
    sketch->connectContainer(4, 6);

    return sketch;
}

MachineGraph* GraphTest::makeImposibleSketch() {
    MachineGraph* sketch = new MachineGraph("sketchTurbidostat");

    std::shared_ptr<ContainerNodeType> cinlet(new ContainerNodeType(MovementType::continuous, ContainerType::inlet));
    std::shared_ptr<ContainerNodeType> cFlow(new ContainerNodeType(MovementType::continuous, ContainerType::flow));
    std::shared_ptr<ContainerNodeType> convergentSwitch(new ContainerNodeType(MovementType::irrelevant, ContainerType::convergent_switch));
    std::shared_ptr<ContainerNodeType> bidirectionalT(new ContainerNodeType(MovementType::continuous, ContainerType::bidirectional_switch));
    std::shared_ptr<ContainerNodeType> cnvSwitchInlet(new ContainerNodeType(MovementType::continuous, ContainerType::convergent_switch_inlet));
    std::shared_ptr<ContainerNodeType> dvrSwitchSink(new ContainerNodeType(MovementType::continuous, ContainerType::divergent_switch_sink));

    sketch->addContainer(1, cinlet, 100.0);
    sketch->addContainer(2, dvrSwitchSink, 100.0);
    sketch->addContainer(3, cFlow, 100.0);
    sketch->addContainer(4, cinlet, 100.0);
    sketch->addContainer(5, cnvSwitchInlet, 100.0);
    sketch->addContainer(6, bidirectionalT, 100.0);
    sketch->addContainer(7, convergentSwitch, 100.0);
    sketch->addContainer(8, cinlet, 100.0);

    sketch->connectContainer(1, 5);
    sketch->connectContainer(5, 7);
    sketch->connectContainer(2, 5);
    sketch->connectContainer(2, 3);
    sketch->connectContainer(6, 2);
    sketch->connectContainer(6, 7);
    sketch->connectContainer(3, 6);
    sketch->connectContainer(4, 6);
    sketch->connectContainer(8, 6);

    return sketch;
}

ExecutableMachineGraph* GraphTest::makeMappingMachine(int communications,
                                                      std::unique_ptr<CommandSender> exec,
                                                      std::unique_ptr<CommandSender> test)
{
    ExecutableMachineGraph* machine = new ExecutableMachineGraph(
                "mappingMachine", std::move(exec), std::move(test));

    std::unordered_map<std::string, std::string> paramsc = {{"address","46"},
                                                            {"closePos","0"}};
    std::shared_ptr<Control> control(
                new ControlPlugin(communications,"v1", "Evoprog4WayValve", paramsc));

    std::unordered_map<std::string, std::string> paramse = {{"address","7"},
                                                            {"direction","0"}};
    std::shared_ptr<Extractor> cExtractor(
                new ExtractorPlugin(communications,"p1", "EvoprogV2Pump", paramse));

    std::unordered_map<std::string, std::string> paramsi;
    std::shared_ptr<Injector> dummyInjector(
                new InjectorPlugin(communications, "dummy", "EvoprogDummyInjector", paramsi));

    std::unordered_map<std::string, std::string> paramso = {{"pinNumber","14"}};
    std::shared_ptr<ODSensor> sensor(new ODSensorPlugin(communications, "sensor1", "EvoprogOdSensor", paramso));

    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet1 = std::make_shared<InletContainer>(1, 100.0, cExtractor);
    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet2 = std::make_shared<DivergentSwitchSink>(2, 100.0, dummyInjector, cExtractor, control);
    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet3 = std::make_shared<FlowContainer>(3, 100.0, cExtractor, dummyInjector);
    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet4 = std::make_shared<InletContainer>(4, 100.0, cExtractor);


    ExecutableMachineGraph::ExecutableContainerNodePtr cSwtInlet5 = std::make_shared<ConvergentSwitchInlet>(5, 100.0,
                                                                                                            dummyInjector, cExtractor, control);
    ExecutableMachineGraph::ExecutableContainerNodePtr cSwtInlet6 = std::make_shared<BidirectionalSwitch>(6, 100.0,
                                                                                                          cExtractor, dummyInjector, control, control);
    cSwtInlet6->setOd(sensor);
    ExecutableMachineGraph::ExecutableContainerNodePtr cSwich7 = std::make_shared<ConvergentSwitch>(7, 100.0, dummyInjector, control);

    machine->addContainer(cInlet1);
    machine->addContainer(cInlet2);
    machine->addContainer(cInlet3);
    machine->addContainer(cInlet4);
    machine->addContainer(cSwtInlet5);
    machine->addContainer(cSwtInlet6);
    machine->addContainer(cSwich7);

    machine->connectExecutableContainer(1, 5);
    machine->connectExecutableContainer(2, 5);
    machine->connectExecutableContainer(3, 6);
    machine->connectExecutableContainer(4, 6);
    machine->connectExecutableContainer(5, 7);
    machine->connectExecutableContainer(6, 7);
    machine->connectExecutableContainer(6, 2);
    machine->connectExecutableContainer(2, 3);

    return machine;
}

ProtocolGraph* GraphTest::makeTimeProtocol()
{

    AutoEnumerate serial;
    ProtocolGraph* protocol = new ProtocolGraph("simpleProtocol");

    std::shared_ptr<ComparisonOperable> tautology(new Tautology());
    std::shared_ptr<MathematicOperable> num1(new ConstantNumber(0.001));
    std::shared_ptr<MathematicOperable> num60000(new ConstantNumber(60000));
    std::shared_ptr<MathematicOperable> num65(new ConstantNumber(65));

    ProtocolGraph::ProtocolNodePtr op1 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num65); //loadContainer(1, 1000ml)

    protocol->addOperation(op1);

    std::shared_ptr<VariableEntry> time(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<MathematicOperable> mtime(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<ComparisonOperable> comp2in(
        new SimpleComparison(false, mtime, comparison::less_equal, num60000));
    ProtocolGraph::ProtocolNodePtr loop1 = std::make_shared<LoopNode>(serial.getNextValue(), comp2in); //while ( t <= 60s)

    protocol->addOperation(loop1);
    protocol->connectOperation(op1, loop1, tautology);

    ProtocolGraph::ProtocolNodePtr op2 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 1, 2, num1);
    ProtocolGraph::ProtocolNodePtr op3 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 2, 3, num1);

    protocol->addOperation(op2);
    protocol->connectOperation(loop1, op2, comp2in);
    protocol->addOperation(op3);
    protocol->connectOperation(op2, op3, tautology);
    ProtocolGraph::ProtocolNodePtr timeStep = std::make_shared<TimeStep>(serial.getNextValue(), time);

    protocol->addOperation(timeStep);
    protocol->connectOperation(op3, timeStep, tautology);
    protocol->connectOperation(timeStep, loop1, tautology);

    protocol->setStartNode(op1->getContainerId());
    return protocol;
}

QTEST_MAIN(GraphTest)

#include "tst_graph.moc"
