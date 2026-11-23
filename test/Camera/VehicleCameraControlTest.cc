#include "VehicleCameraControlTest.h"

#include <cstring>
#include <memory>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QScopeGuard>
#include <QtCore/QTemporaryDir>
#include <QtTest/QSignalSpy>

#include "AppSettings.h"
#include "LinkManager.h"
#include "MavlinkCameraControlInterface.h"
#include "MockConfiguration.h"
#include "MockLink.h"
#include "MultiVehicleManager.h"
#include "QGCCameraManager.h"
#include "SettingsManager.h"
#include "Vehicle.h"
#include "VehicleCameraControl.h"

namespace {

class CameraControlProbe : public VehicleCameraControl
{
public:
    using VehicleCameraControl::VehicleCameraControl;

    QString cacheFile() const { return _cacheFile; }

    bool paramComplete() const { return _paramComplete; }
};

MockConfiguration* newMockConfiguration(const QString& name)
{
    auto* mockConfig = new MockConfiguration(name);
    mockConfig->setFirmwareType(MAV_AUTOPILOT_PX4);
    mockConfig->setVehicleType(MAV_TYPE_QUADROTOR);
    mockConfig->setDynamic(true);
    return mockConfig;
}

/// Points the app save path at rootPath until the returned guard goes out of scope.
[[nodiscard]] auto overrideSavePath(const QString& rootPath)
{
    Fact* savePathFact = SettingsManager::instance()->appSettings()->savePath();
    const QVariant originalSavePath = savePathFact->rawValue();
    savePathFact->setRawValue(rootPath);
    return qScopeGuard([savePathFact, originalSavePath]() { savePathFact->setRawValue(originalSavePath); });
}

constexpr char kActionDefinition[] = R"(<?xml version="1.0" encoding="UTF-8" ?>
<mavlinkcamera>
    <definition version="1">
        <model>ActionCam</model>
        <vendor>Acme</vendor>
    </definition>
    <parameters>
        <parameter name="CAM_TESTMODE" type="uint32" default="0" writeonly="1">
            <description>Test Mode</description>
            <options>
                <option name="Photo" value="0" />
                <option name="Video" value="1">
                    <exclusions>
                        <exclude>CAM_AFONCE</exclude>
                        <exclude>TXT_NOTE</exclude>
                    </exclusions>
                </option>
            </options>
        </parameter>
        <text name="TXT_HEADER" size="large" bold="1" align="center">
            Header
        </text>
        <action name="CAM_AFONCE" type="uint8">
            <description>Autofocus</description>
            <buttons><button value="1" label="Focus Now" /></buttons>
        </action>
        <text>Unnamed note</text>
        <text name="TXT_NOTE" size="huge" align="middle">Note</text>
        <parameter name="CAM_PALETTE" type="uint8" default="0" writeonly="1">
            <description>Palette</description>
            <options>
                <option name="White" value="0" />
                <option name="Black" value="1" />
            </options>
        </parameter>
        <action name="CAM_QUICKPAL" type="uint8" highlight="1">
            <description>Quick Palette</description>
            <buttons>
                <button value="0" label="White" />
                <button value="1" label="Black" />
            </buttons>
        </action>
        <parameter name="CAM_HIDDEN" type="uint8" default="0" control="0" writeonly="1">
            <description>Hidden</description>
        </parameter>
        <action name="CAM_FACTRESET" type="uint8">
            <description>Factory Reset</description>
            <buttons>
                <button value="1">
                    <confirm title="Factory Reset" yes="Reset" no="Keep">All settings will be lost.</confirm>
                </button>
            </buttons>
        </action>
        <action name="CAM_BADVALUE" type="uint8">
            <description>Bad Value</description>
            <buttons>
                <button value="300" label="Too Big" />
                <button value="2" label="Fine" />
            </buttons>
        </action>
        <action name="CAM_NOBUTTONS" type="uint8">
            <description>No Buttons</description>
            <buttons><button value="abc" /></buttons>
        </action>
        <action name="CAM_BADTYPE" type="button">
            <description>Bad Type</description>
            <buttons><button value="1" /></buttons>
        </action>
        <action name="CAM_PALETTE" type="uint8">
            <description>Duplicate Name</description>
            <buttons><button value="1" /></buttons>
        </action>
        <text name="CAM_TESTMODE">Duplicate Name</text>
    </parameters>
</mavlinkcamera>
)";

mavlink_param_ext_ack_t paramExtAck(const char* paramId, uint8_t result, uint8_t value)
{
    mavlink_param_ext_ack_t ack{};
    (void) qstrncpy(ack.param_id, paramId, sizeof(ack.param_id));
    ack.param_value[0] = static_cast<char>(value);
    ack.param_type = MAV_PARAM_EXT_TYPE_UINT8;
    ack.param_result = result;
    return ack;
}

}  // namespace

/// A camera of model "ActionCam" whose definition is served from the definition cache. The
/// definition is loaded by the time the camera is constructed.
std::unique_ptr<VehicleCameraControl> VehicleCameraControlTest::_cameraWithDefinition(const QByteArray& definition)
{
    mavlink_camera_information_t info{};
    memcpy(info.vendor_name, "Acme", 5);
    memcpy(info.model_name, "ActionCam", 10);
    static constexpr char uri[] = "http://127.0.0.1/actioncam.xml";
    memcpy(info.cam_definition_uri, uri, sizeof(uri));

    const QString cacheDir = SettingsManager::instance()->appSettings()->parameterSavePath();
    QFile cacheFile(QDir(cacheDir).filePath(
        VehicleCameraControl::cacheFileToken(QStringLiteral("Acme"), QStringLiteral("ActionCam"), 0) +
        QStringLiteral(".xml")));
    if (!cacheFile.open(QIODevice::WriteOnly) || (cacheFile.write(definition) < 0)) {
        return nullptr;
    }
    cacheFile.close();

    return std::make_unique<CameraControlProbe>(&info, _vehicle, MAV_COMP_ID_CAMERA6);
}

/// A camera whose definition is kActionDefinition. The parse warnings for its invalid actions
/// and texts are verified here.
std::unique_ptr<VehicleCameraControl> VehicleCameraControlTest::_actionCamera()
{
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Invalid size for text \"TXT_NOTE\" : \"huge\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Invalid align for text \"TXT_NOTE\" : \"middle\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Invalid button value for action \"CAM_BADVALUE\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Invalid button value for action \"CAM_NOBUTTONS\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Action \"CAM_NOBUTTONS\" has no valid buttons")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Unsupported type for action \"CAM_BADTYPE\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Duplicate action name: \"CAM_PALETTE\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Duplicate text name: \"CAM_TESTMODE\"")));

    auto camera = _cameraWithDefinition(kActionDefinition);

    for (int i = 0; i < 8; i++) {
        verifyExpectedLogMessage();
    }
    if (camera) {
        // The camera reports the highlighted action's value, which answers the definition's request for it
        mavlink_param_ext_value_t value{};
        (void) qstrncpy(value.param_id, "CAM_QUICKPAL", sizeof(value.param_id));
        value.param_type = MAV_PARAM_EXT_TYPE_UINT8;
        camera->handleParamExtValue(value);
    }
    return camera;
}

void VehicleCameraControlTest::_testDefinitionLoad()
{
    _connectVehicle(newMockConfiguration(QStringLiteral("CameraDefinitionLoadTest")));
    if (QTest::currentTestFailed()) {
        return;
    }
    QTemporaryDir saveRoot;
    QVERIFY(saveRoot.isValid());
    const auto restoreSavePath = overrideSavePath(saveRoot.path());

    auto camera = _actionCamera();
    QVERIFY(camera);

    // Actions and texts are listed with the parameters, in definition order. A parameter without a
    // control isn't listed, and invalid actions and duplicate names are dropped.
    const QStringList expectedSettings = {
        QStringLiteral("CAM_TESTMODE"), QStringLiteral("TXT_HEADER"),    QStringLiteral("CAM_AFONCE"),
        QStringLiteral("#text2"),       QStringLiteral("TXT_NOTE"),      QStringLiteral("CAM_PALETTE"),
        QStringLiteral("CAM_QUICKPAL"), QStringLiteral("CAM_FACTRESET"), QStringLiteral("CAM_BADVALUE")};
    QCOMPARE(camera->activeSettings(), expectedSettings);
    QCOMPARE(camera->elementInfo(QStringLiteral("CAM_PALETTE")).value(QStringLiteral("type")).toString(),
             QStringLiteral("parameter"));

    const QVariantMap focus = camera->elementInfo(QStringLiteral("CAM_AFONCE"));
    QCOMPARE(focus.value(QStringLiteral("type")).toString(), QStringLiteral("action"));
    QCOMPARE(focus.value(QStringLiteral("description")).toString(), QStringLiteral("Autofocus"));
    const QVariantMap focusButton = focus.value(QStringLiteral("buttons")).toList().value(0).toMap();
    QCOMPARE(focusButton.value(QStringLiteral("label")).toString(), QStringLiteral("Focus Now"));
    QCOMPARE(focusButton.value(QStringLiteral("value")).toUInt(), 1u);
    QCOMPARE(focusButton.value(QStringLiteral("confirm")).toBool(), false);

    const QVariantMap resetButton =
        camera->elementInfo(QStringLiteral("CAM_FACTRESET")).value(QStringLiteral("buttons")).toList().value(0).toMap();
    QCOMPARE(resetButton.value(QStringLiteral("confirm")).toBool(), true);
    QCOMPARE(resetButton.value(QStringLiteral("confirmTitle")).toString(), QStringLiteral("Factory Reset"));
    QCOMPARE(resetButton.value(QStringLiteral("confirmText")).toString(), QStringLiteral("All settings will be lost."));
    QCOMPARE(resetButton.value(QStringLiteral("confirmYes")).toString(), QStringLiteral("Reset"));
    QCOMPARE(resetButton.value(QStringLiteral("confirmNo")).toString(), QStringLiteral("Keep"));

    // Only the button whose value fits the type is kept
    const QVariantList badValueButtons =
        camera->elementInfo(QStringLiteral("CAM_BADVALUE")).value(QStringLiteral("buttons")).toList();
    QCOMPARE(badValueButtons.size(), 1);
    QCOMPARE(badValueButtons[0].toMap().value(QStringLiteral("label")).toString(), QStringLiteral("Fine"));

    // The text is trimmed; invalid attribute values fall back to the defaults
    const QVariantMap header = camera->elementInfo(QStringLiteral("TXT_HEADER"));
    QCOMPARE(header.value(QStringLiteral("type")).toString(), QStringLiteral("text"));
    QCOMPARE(header.value(QStringLiteral("text")).toString(), QStringLiteral("Header"));
    QCOMPARE(header.value(QStringLiteral("size")).toString(), QStringLiteral("large"));
    QCOMPARE(header.value(QStringLiteral("bold")).toBool(), true);
    QCOMPARE(header.value(QStringLiteral("align")).toString(), QStringLiteral("center"));
    const QVariantMap note = camera->elementInfo(QStringLiteral("TXT_NOTE"));
    QCOMPARE(note.value(QStringLiteral("size")).toString(), QStringLiteral("default"));
    QCOMPARE(note.value(QStringLiteral("align")).toString(), QStringLiteral("left"));

    // Actions and texts are excluded like parameters
    mavlink_param_ext_value_t value{};
    (void) qstrncpy(value.param_id, "CAM_TESTMODE", sizeof(value.param_id));
    value.param_type = MAV_PARAM_EXT_TYPE_UINT32;
    value.param_value[0] = 1;
    camera->handleParamExtValue(value);
    QVERIFY(!camera->activeSettings().contains(QStringLiteral("CAM_AFONCE")));
    QVERIFY(!camera->activeSettings().contains(QStringLiteral("TXT_NOTE")));
}

void VehicleCameraControlTest::_testDefinitionGroups()
{
    _connectVehicle(newMockConfiguration(QStringLiteral("CameraGroupsTest")));
    if (QTest::currentTestFailed()) {
        return;
    }
    QTemporaryDir saveRoot;
    QVERIFY(saveRoot.isValid());
    const auto restoreSavePath = overrideSavePath(saveRoot.path());

    static constexpr char definition[] = R"(<?xml version="1.0" encoding="UTF-8" ?>
<mavlinkcamera>
    <definition version="1">
        <model>ActionCam</model>
        <vendor>Acme</vendor>
    </definition>
    <parameters>
        <parameter name="CAM_TESTMODE" type="uint32" default="0" writeonly="1">
            <description>Test Mode</description>
            <options>
                <option name="Photo" value="0" />
                <option name="Video" value="1">
                    <exclusions><exclude>GRP_PHOTO</exclude></exclusions>
                </option>
                <option name="NoLoose" value="2">
                    <exclusions><exclude>CAM_LOOSE</exclude></exclusions>
                </option>
            </options>
        </parameter>
        <group name="GRP_PHOTO" title="Photo" color="#80FF0000" radius="8.5">
            <text>Hint</text>
            <parameter name="CAM_RES" type="uint8" default="0" writeonly="1">
                <description>Resolution</description>
            </parameter>
            <group name="GRP_INNER">
                <parameter name="CAM_INNER" type="uint8" default="0" writeonly="1">
                    <description>Inner</description>
                </parameter>
            </group>
        </group>
        <group color="notacolor" radius="-2">
            <parameter name="CAM_LOOSE" type="uint8" default="0" writeonly="1">
                <description>Loose</description>
            </parameter>
        </group>
        <group name="GRP_EMPTY" title="Nothing" />
        <parameter name="CAM_AFTER" type="uint8" default="0" writeonly="1">
            <description>After</description>
        </parameter>
    </parameters>
</mavlinkcamera>
)";
    expectLogMessage(
        "Camera.VehicleCameraControl", QtWarningMsg,
        QRegularExpression(QStringLiteral("Groups can't be nested, the content belongs to group: \"GRP_PHOTO\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Invalid color for group \"#group2\" : \"notacolor\"")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Invalid radius for group \"#group2\" : \"-2\"")));
    auto camera = _cameraWithDefinition(definition);
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    QVERIFY(camera);

    // A group is listed before its entries; a nested group's entries belong to the outer one,
    // and a group with nothing in it isn't listed
    const QStringList expectedSettings = {QStringLiteral("CAM_TESTMODE"), QStringLiteral("GRP_PHOTO"),
                                          QStringLiteral("#text1"),       QStringLiteral("CAM_RES"),
                                          QStringLiteral("CAM_INNER"),    QStringLiteral("#group2"),
                                          QStringLiteral("CAM_LOOSE"),    QStringLiteral("CAM_AFTER")};
    QCOMPARE(camera->activeSettings(), expectedSettings);

    const QVariantMap photo = camera->elementInfo(QStringLiteral("GRP_PHOTO"));
    QCOMPARE(photo.value(QStringLiteral("type")).toString(), QStringLiteral("group"));
    QCOMPARE(photo.value(QStringLiteral("title")).toString(), QStringLiteral("Photo"));
    QCOMPARE(photo.value(QStringLiteral("color")).toString(), QStringLiteral("#80FF0000"));
    QCOMPARE(photo.value(QStringLiteral("radius")).toDouble(), 8.5);
    QCOMPARE(camera->elementInfo(QStringLiteral("CAM_INNER")).value(QStringLiteral("group")).toString(),
             QStringLiteral("GRP_PHOTO"));
    QVERIFY(!camera->elementInfo(QStringLiteral("CAM_AFTER")).contains(QStringLiteral("group")));

    // An invalid color or radius leaves the standard one
    const QVariantMap loose = camera->elementInfo(QStringLiteral("#group2"));
    QCOMPARE(loose.value(QStringLiteral("color")).toString(), QString());
    QVERIFY(loose.value(QStringLiteral("radius")).toDouble() < 0);

    mavlink_param_ext_value_t value{};
    (void) qstrncpy(value.param_id, "CAM_TESTMODE", sizeof(value.param_id));
    value.param_type = MAV_PARAM_EXT_TYPE_UINT32;

    // Excluding a group excludes everything in it
    value.param_value[0] = 1;
    camera->handleParamExtValue(value);
    QCOMPARE(camera->activeSettings(), (QStringList{QStringLiteral("CAM_TESTMODE"), QStringLiteral("#group2"),
                                                    QStringLiteral("CAM_LOOSE"), QStringLiteral("CAM_AFTER")}));

    // A group with all its entries excluded isn't shown either
    value.param_value[0] = 2;
    camera->handleParamExtValue(value);
    QVERIFY(!camera->activeSettings().contains(QStringLiteral("#group2")));
    QVERIFY(camera->activeSettings().contains(QStringLiteral("GRP_PHOTO")));
}

void VehicleCameraControlTest::_testActionSendsEveryClickAndNeverResends()
{
    _connectVehicle(newMockConfiguration(QStringLiteral("CameraActionSendTest")));
    if (QTest::currentTestFailed()) {
        return;
    }
    QTemporaryDir saveRoot;
    QVERIFY(saveRoot.isValid());
    const auto restoreSavePath = overrideSavePath(saveRoot.path());

    auto camera = _actionCamera();
    QVERIFY(camera);
    _mockLink->clearReceivedMavlinkMessageCounts();

    camera->triggerAction(QStringLiteral("CAM_AFONCE"), 0);
    QCOMPARE(camera->pendingActions(), QStringList{QStringLiteral("CAM_AFONCE")});
    QTRY_COMPARE_WITH_TIMEOUT(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_EXT_SET), 1,
                              TestTimeout::longMs());

    mavlink_message_t message{};
    QVERIFY(_mockLink->lastReceivedMavlinkMessage(MAVLINK_MSG_ID_PARAM_EXT_SET, message));
    mavlink_param_ext_set_t paramSet{};
    mavlink_msg_param_ext_set_decode(&message, &paramSet);
    QCOMPARE(QGCMAVLink::boundedStringField(paramSet.param_id), QStringLiteral("CAM_AFONCE"));
    QCOMPARE(paramSet.param_type, static_cast<uint8_t>(MAV_PARAM_EXT_TYPE_UINT8));
    QCOMPARE(static_cast<uint8_t>(paramSet.param_value[0]), static_cast<uint8_t>(1));
    QCOMPARE(paramSet.target_component, static_cast<uint8_t>(MAV_COMP_ID_CAMERA6));

    // A click while the camera hasn't answered sends nothing
    camera->triggerAction(QStringLiteral("CAM_AFONCE"), 0);

    camera->handleParamExtAck(paramExtAck("CAM_AFONCE", PARAM_ACK_ACCEPTED, 1));
    QVERIFY(camera->pendingActions().isEmpty());

    // The same value is sent again on the next click
    camera->triggerAction(QStringLiteral("CAM_AFONCE"), 0);
    QTRY_COMPARE_WITH_TIMEOUT(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_EXT_SET), 2,
                              TestTimeout::longMs());

    // Unanswered, the action is reported as failed and never resent. Meanwhile the requests the
    // camera sent on creation give up too, as no camera answers them.
    ignoreLogMessage("Vehicle.MavCommandQueue", QtWarningMsg,
                     QRegularExpression("Giving up sending command after max retries:"));
    expectLogMessage("Camera.QGCCameraParamIO", QtWarningMsg,
                     QRegularExpression(QStringLiteral("No response for action")));
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Camera action failed")));
    expectAppMessage(QRegularExpression(QStringLiteral("Autofocus: no response from camera")));
    QTRY_VERIFY_WITH_TIMEOUT(camera->pendingActions().isEmpty(), TestTimeout::longMs());
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    QCOMPARE(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_EXT_SET), 2);

    // A late answer is ignored: it neither reports the action again nor sends anything
    camera->handleParamExtAck(paramExtAck("CAM_AFONCE", PARAM_ACK_FAILED, 1));
    QVERIFY(camera->pendingActions().isEmpty());
    QCOMPARE(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_EXT_SET), 2);
}

void VehicleCameraControlTest::_testActionAnswerUpdatesHighlightAndReportsFailure()
{
    _connectVehicle(newMockConfiguration(QStringLiteral("CameraActionAnswerTest")));
    if (QTest::currentTestFailed()) {
        return;
    }
    QTemporaryDir saveRoot;
    QVERIFY(saveRoot.isValid());
    const auto restoreSavePath = overrideSavePath(saveRoot.path());

    auto camera = _actionCamera();
    QVERIFY(camera);
    // The value the camera reported for the highlighted action
    Fact* palette = camera->getFact(QStringLiteral("CAM_QUICKPAL"));
    QVERIFY(palette);
    QCOMPARE(palette->rawValue().toUInt(), 0u);

    // In progress keeps the action waiting; accepted sets the value its highlight follows
    camera->triggerAction(QStringLiteral("CAM_QUICKPAL"), 1);
    camera->handleParamExtAck(paramExtAck("CAM_QUICKPAL", PARAM_ACK_IN_PROGRESS, 0));
    QCOMPARE(camera->pendingActions(), QStringList{QStringLiteral("CAM_QUICKPAL")});
    camera->handleParamExtAck(paramExtAck("CAM_QUICKPAL", PARAM_ACK_ACCEPTED, 1));
    QVERIFY(camera->pendingActions().isEmpty());
    QCOMPARE(palette->rawValue().toUInt(), 1u);

    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Camera action failed")));
    expectAppMessage(QRegularExpression(QStringLiteral("Factory Reset: rejected by camera")));
    camera->triggerAction(QStringLiteral("CAM_FACTRESET"), 0);
    camera->handleParamExtAck(paramExtAck("CAM_FACTRESET", PARAM_ACK_FAILED, 0));
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    QVERIFY(camera->pendingActions().isEmpty());
}

void VehicleCameraControlTest::_testNameFieldReadIsBounded()
{
    // CAMERA_INFORMATION.vendor_name and model_name are uint8_t[32] filled straight from
    // the wire. MAVLink does not guarantee a NUL, so reading them as C strings runs past
    // the array into whatever follows it in the struct.
    struct
    {
        uint8_t vendor[32];
        uint8_t model[32];
    } wire{};

    memset(wire.vendor, 'A', sizeof(wire.vendor));
    memcpy(wire.model, "SHOULD_NOT_APPEAR", 18);

    const QString bounded = QGCMAVLink::boundedStringField(wire.vendor);

    QCOMPARE(bounded.length(), 32);
    QVERIFY(!bounded.contains(QLatin1String("SHOULD_NOT_APPEAR")));

    // A short, properly terminated field must still read normally.
    memset(&wire, 0, sizeof(wire));
    memcpy(wire.vendor, "Sony", 5);
    QCOMPARE(QGCMAVLink::boundedStringField(wire.vendor), QStringLiteral("Sony"));

    // QGC has always decoded these names as UTF-8, so a non-ASCII name must not turn into Latin-1 mojibake.
    static constexpr char utf8Name[] = "C\xC3\xA1mara";
    memset(&wire, 0, sizeof(wire));
    memcpy(wire.vendor, utf8Name, sizeof(utf8Name));
    QCOMPARE(QGCMAVLink::boundedStringField(wire.vendor), QStringLiteral("C\u00e1mara"));
}

void VehicleCameraControlTest::_testCameraNamesCannotSteerCachePath()
{
    // The vendor/model strings are formatted into the camera-definition cache file name.
    // Whatever they contain, the result has to stay a single component inside the cache
    // directory.
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());
    const QString base = QDir::cleanPath(QDir(tempDir.path()).absolutePath());

    const QList<QPair<QString, QString>> hostile = {
        {QStringLiteral("../../../Desktop/QGC01_PWNED"), QStringLiteral("marker")},
        {QStringLiteral("..\\..\\..\\Desktop\\PWNED"), QStringLiteral("marker")},
        {QStringLiteral("C:\\Windows\\Temp\\PWNED"), QStringLiteral("marker")},
        {QStringLiteral("/etc/cron.d/PWNED"), QStringLiteral("marker")},
        {QStringLiteral("T.txt:"), QStringLiteral("s")},
        {QStringLiteral("vendor"), QStringLiteral("../../evil")},
        {QStringLiteral(".."), QStringLiteral("..")},
        {QString(), QStringLiteral("marker")},
    };

    for (const auto& testCase : hostile) {
        const QString token = VehicleCameraControl::cacheFileToken(testCase.first, testCase.second, 1);
        const QString candidateFilePath = QDir(base).filePath(token + QStringLiteral(".xml"));

        QVERIFY2(VehicleCameraControl::pathIsInside(base, candidateFilePath), qPrintable(candidateFilePath));

        // It must be one component: no separator, and no drive or NTFS stream qualifier.
        QVERIFY2(!token.contains(QLatin1Char('/')), qPrintable(token));
        QVERIFY2(!token.contains(QLatin1Char('\\')), qPrintable(token));
        QVERIFY2(!token.contains(QLatin1Char(':')), qPrintable(token));
    }

    // An unset cache directory must be refused rather than becoming a write at "/".
    QVERIFY(!VehicleCameraControl::pathIsInside(QString(), QStringLiteral("/v_m_001.xml")));
}

void VehicleCameraControlTest::_testOrdinaryCameraNamesUnchanged()
{
    // Containment must not mangle a real camera's cache file name beyond replacing
    // characters that cannot appear in one.
    QCOMPARE(VehicleCameraControl::pathSafeNameToken(QStringLiteral("Sony")), QStringLiteral("Sony"));
    QCOMPARE(VehicleCameraControl::pathSafeNameToken(QStringLiteral("ILCE-7RM4")), QStringLiteral("ILCE-7RM4"));
    QCOMPARE(VehicleCameraControl::pathSafeNameToken(QStringLiteral("Skynode.v2")), QStringLiteral("Skynode.v2"));
    QCOMPARE(VehicleCameraControl::pathSafeNameToken(QStringLiteral("FLIR Boson")), QStringLiteral("FLIR_Boson"));
    QCOMPARE(VehicleCameraControl::pathSafeNameToken(QString()), QStringLiteral("unknown"));

    const QString token = VehicleCameraControl::cacheFileToken(QStringLiteral("Sony"), QStringLiteral("ILCE-7RM4"), 7);
    QVERIFY2(token.startsWith(QStringLiteral("Sony_ILCE-7RM4_")), qPrintable(token));
    QVERIFY2(token.endsWith(QStringLiteral("_007")), qPrintable(token));
    QCOMPARE(token, VehicleCameraControl::cacheFileToken(QStringLiteral("Sony"), QStringLiteral("ILCE-7RM4"), 7));
}

void VehicleCameraControlTest::_testDistinctCameraNamesDoNotShareCacheFile()
{
    // pathSafeNameToken() is lossy; distinct cameras must still get distinct cache files.
    const QList<QPair<QPair<QString, QString>, QPair<QString, QString>>> collidingAfterSanitising = {
        {{QStringLiteral("FLIR"), QStringLiteral("Boson 640")}, {QStringLiteral("FLIR"), QStringLiteral("Boson_640")}},
        {{QStringLiteral("FLIR"), QStringLiteral("Boson/640")}, {QStringLiteral("FLIR"), QStringLiteral("Boson:640")}},
        {{QStringLiteral("\u5927\u7586"), QStringLiteral("Cam")},
         {QStringLiteral("\u6d77\u5eb7"), QStringLiteral("Cam")}},
        {{QStringLiteral(".Cam"), QStringLiteral("X")}, {QStringLiteral("Cam"), QStringLiteral("X")}},
        {{QStringLiteral("A_B"), QStringLiteral("C")}, {QStringLiteral("A"), QStringLiteral("B_C")}},
        {{QStringLiteral("Sony"), QStringLiteral("X")}, {QStringLiteral("sony"), QStringLiteral("X")}},
    };

    for (const auto& pair : collidingAfterSanitising) {
        const QString first = VehicleCameraControl::cacheFileToken(pair.first.first, pair.first.second, 1);
        const QString second = VehicleCameraControl::cacheFileToken(pair.second.first, pair.second.second, 1);
        QVERIFY2(first.compare(second, Qt::CaseInsensitive) != 0, qPrintable(first + QStringLiteral(" == ") + second));
    }
}

void VehicleCameraControlTest::_connectVehicle(MockConfiguration* mockConfig)
{
    QSignalSpy spyVehicle(MultiVehicleManager::instance(), &MultiVehicleManager::activeVehicleChanged);
    QVERIFY(spyVehicle.isValid());

    SharedLinkConfigurationPtr linkConfig = LinkManager::instance()->addConfiguration(mockConfig);
    QVERIFY(LinkManager::instance()->createConnectedLink(linkConfig));
    QVERIFY2(UnitTest::waitForSignal(spyVehicle, TestTimeout::longMs(), QStringLiteral("activeVehicleChanged")),
             "Timeout waiting for vehicle connection");

    _vehicle = MultiVehicleManager::instance()->activeVehicle();
    QVERIFY(_vehicle);
    _mockLink = qobject_cast<MockLink*>(linkConfig->link());
    QVERIFY(_mockLink);

    if (!_vehicle->isInitialConnectComplete()) {
        QSignalSpy spyConnect(_vehicle, &Vehicle::initialConnectComplete);
        QVERIFY(spyConnect.isValid());
        QVERIFY2(UnitTest::waitForSignal(spyConnect, TestTimeout::longMs(), QStringLiteral("initialConnectComplete")),
                 "Timeout waiting for initial connect");
    }
}

MavlinkCameraControlInterface* VehicleCameraControlTest::_firstCamera()
{
    QGCCameraManager* const cameraManager = _vehicle->cameraManager();
    // MockLinkCamera creates two cameras.
    if (!cameraManager || !waitForCondition([cameraManager] { return cameraManager->cameras()->count() >= 2; },
                                            TestTimeout::longMs(), u"two cameras discovered")) {
        return nullptr;
    }
    for (int i = 0; i < cameraManager->cameras()->count(); i++) {
        auto* const camera = qobject_cast<MavlinkCameraControlInterface*>(cameraManager->cameras()->get(i));
        if (camera && (camera->compID() == MAV_COMP_ID_CAMERA)) {
            return camera;
        }
    }
    return nullptr;
}

void VehicleCameraControlTest::_testConstructorBuildsCacheFileFromBoundedNames()
{
    _connectVehicle(newMockConfiguration(QStringLiteral("CameraCacheFileTest")));
    if (QTest::currentTestFailed()) {
        return;
    }

    const QString cacheDir = SettingsManager::instance()->appSettings()->parameterSavePath();
    QVERIFY(!cacheDir.isEmpty());

    // model_name is completely full and lens_id, which follows it, is non-zero, so an
    // unbounded read would pick up the extra byte.
    mavlink_camera_information_t info{};
    memcpy(info.vendor_name, "../../evil", 11);
    memset(info.model_name, 'M', sizeof(info.model_name));
    info.lens_id = 'X';
    info.cam_definition_version = 3;

    auto camera = std::make_unique<CameraControlProbe>(&info, _vehicle, MAV_COMP_ID_CAMERA6);

    const QString expectedModel(static_cast<qsizetype>(sizeof(info.model_name)), QLatin1Char('M'));
    QCOMPARE(camera->vendor(), QStringLiteral("../../evil"));
    QCOMPARE(camera->modelName(), expectedModel);

    const QString expectedToken = VehicleCameraControl::cacheFileToken(QStringLiteral("../../evil"), expectedModel, 3);
    QCOMPARE(camera->cacheFile(), QDir::cleanPath(QDir(cacheDir).filePath(expectedToken + QStringLiteral(".xml"))));
    QVERIFY(VehicleCameraControl::pathIsInside(cacheDir, camera->cacheFile()));

    // No definition URI, so the camera initialises as a basic camera straight away.
    QVERIFY(camera->paramComplete());
}

void VehicleCameraControlTest::_testFtpDefinitionDownloadRefusedFallsBack()
{
    _connectVehicle(newMockConfiguration(QStringLiteral("CameraFtpFallbackTest")));
    if (QTest::currentTestFailed()) {
        return;
    }

    // parameterSavePath() is empty when the save root does not exist. Point the save path
    // at a temporary root and remove it, rather than clearing the setting, which would
    // make AppSettings create its subdirectories in the working directory.
    QTemporaryDir missingRoot;
    QVERIFY(missingRoot.isValid());
    const auto restoreSavePath = overrideSavePath(missingRoot.path());
    QVERIFY(missingRoot.remove());
    QVERIFY(SettingsManager::instance()->appSettings()->parameterSavePath().isEmpty());

    // cam_definition_uri is completely full and the bytes after it are non-zero, so an
    // unbounded read would run past the array into them.
    struct
    {
        mavlink_camera_information_t info;
        char tail[4];
    } wire{};

    memcpy(wire.info.vendor_name, "Acme", 5);
    memcpy(wire.info.model_name, "Cam", 4);
    memset(wire.info.cam_definition_uri, 'A', sizeof(wire.info.cam_definition_uri));
    memcpy(wire.info.cam_definition_uri, "mftp://", 7);
    wire.info.gimbal_device_id = 'B';
    wire.info.camera_device_id = 'C';
    memcpy(wire.tail, "ZZZ", 4);

    const qsizetype uriPayloadLength = static_cast<qsizetype>(sizeof(wire.info.cam_definition_uri)) - 7;
    expectLogMessage("Camera.VehicleCameraControl", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Camera definition cache directory is not set")));
    expectLogMessage("Vehicle.FTPManager", QtWarningMsg,
                     QRegularExpression(QStringLiteral("Destination directory is not set")));
    expectLogMessage(
        "Camera.VehicleCameraControl", QtWarningMsg,
        QRegularExpression(
            QStringLiteral("Could not start camera definition ftp download: \"mftp://A{%1}\"$").arg(uriPayloadLength)));

    auto camera = std::make_unique<CameraControlProbe>(&wire.info, _vehicle, MAV_COMP_ID_CAMERA6);

    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
    QVERIFY(camera->cacheFile().isEmpty());

    // The refused download must not leave the camera waiting forever for a definition.
    QVERIFY(camera->paramComplete());
}

void VehicleCameraControlTest::_testFtpDefinitionDownloadIsSavedAsCacheFile_data()
{
    QTest::addColumn<QString>("vehiclePath");
    QTest::addColumn<QString>("compressedSuffix");

    QTest::newRow("plain") << QStringLiteral("/general.json") << QString();
    QTest::newRow("xz") << QStringLiteral("/general.json.xz") << QStringLiteral(".xz");
}

void VehicleCameraControlTest::_testFtpDefinitionDownloadIsSavedAsCacheFile()
{
    // The downloaded file is the cache: _ftpDownloadComplete() marks the definition as cached,
    // so nothing else writes it. Its name, minus any compression suffix, therefore has to be
    // _cacheFile, or the next connect never finds it and downloads it again.
    QFETCH(QString, vehiclePath);
    QFETCH(QString, compressedSuffix);

    _connectVehicle(newMockConfiguration(QStringLiteral("CameraFtpCacheNameTest")));
    if (QTest::currentTestFailed()) {
        return;
    }

    QTemporaryDir saveRoot;
    QVERIFY(saveRoot.isValid());
    const auto restoreSavePath = overrideSavePath(saveRoot.path());
    const QString cacheDir = SettingsManager::instance()->appSettings()->parameterSavePath();
    QVERIFY(QDir(cacheDir).exists());

    // MockLinkFTP serves its JSON metadata at these paths; the content only has to survive
    // the trip, it does not need to be a valid camera definition.
    const QString uri = QStringLiteral("mftp://[;comp=%1]%2").arg(MAV_COMP_ID_AUTOPILOT1).arg(vehiclePath);
    const QByteArray uriBytes = uri.toUtf8();
    mavlink_camera_information_t info{};
    memcpy(info.vendor_name, "Acme", 5);
    memcpy(info.model_name, "Cam", 4);
    QVERIFY(uriBytes.size() < static_cast<qsizetype>(sizeof(info.cam_definition_uri)));
    memcpy(info.cam_definition_uri, uriBytes.constData(), static_cast<size_t>(uriBytes.size()));

    expectLogMessage("default", QtCriticalMsg, QRegularExpression(QStringLiteral("Unable to parse camera definition")));
    // The second message is Qt's own XML parser error text, which can change between Qt
    // versions, so only its presence is checked.
    expectLogMessage("default", QtCriticalMsg, QRegularExpression(QStringLiteral(".+")));

    auto camera = std::make_unique<CameraControlProbe>(&info, _vehicle, MAV_COMP_ID_CAMERA6);
    QTRY_VERIFY_WITH_TIMEOUT(camera->paramComplete(), TestTimeout::longMs());

    verifyExpectedLogMessage();
    verifyExpectedLogMessage();

    const QString expectedCacheFile = QDir::cleanPath(
        QDir(cacheDir).filePath(VehicleCameraControl::cacheFileToken(QStringLiteral("Acme"), QStringLiteral("Cam"), 0) +
                                QStringLiteral(".xml")));
    QCOMPARE(camera->cacheFile(), expectedCacheFile);

    QFile cached(expectedCacheFile);
    QVERIFY2(cached.open(QIODevice::ReadOnly), qPrintable(expectedCacheFile));
    QFile served(QStringLiteral(":MockLink/General.MetaData.json"));
    QVERIFY(served.open(QIODevice::ReadOnly));
    QCOMPARE(cached.readAll(), served.readAll());

    if (!compressedSuffix.isEmpty()) {
        QVERIFY2(QFile::exists(expectedCacheFile + compressedSuffix), qPrintable(expectedCacheFile + compressedSuffix));
    }
}

void VehicleCameraControlTest::initTestCase()
{
    UnitTest::initTestCase();
    MultiVehicleManager::instance()->init();
}

void VehicleCameraControlTest::init()
{
    UnitTest::init();
    _mockLink = nullptr;
    _vehicle = nullptr;
}

void VehicleCameraControlTest::cleanup()
{
    if (_mockLink) {
        QSignalSpy spyDisconnect(MultiVehicleManager::instance(), &MultiVehicleManager::activeVehicleChanged);
        _mockLink->disconnect();
        _mockLink = nullptr;

        if (_vehicle) {
            UnitTest::waitForSignal(spyDisconnect, TestTimeout::longMs(), QStringLiteral("activeVehicleChanged"));
        }
        _vehicle = nullptr;

        UnitTest::settleEventLoopForCleanup();
    }

    dumpFailureContextIfTestFailed(QStringLiteral("cleanup"));
    UnitTest::cleanup();
}

void VehicleCameraControlTest::_testCameraCapFlags_data()
{
    // MockConfiguration camera flags
    QTest::addColumn<bool>("captureVideo");
    QTest::addColumn<bool>("captureImage");
    QTest::addColumn<bool>("hasModes");
    QTest::addColumn<bool>("hasVideoStream");
    QTest::addColumn<bool>("canCaptureImageInVideoMode");
    QTest::addColumn<bool>("canCaptureVideoInImageMode");
    QTest::addColumn<bool>("hasBasicZoom");
    QTest::addColumn<bool>("hasTrackingPoint");
    QTest::addColumn<bool>("hasTrackingRectangle");

    // Expected VehicleCameraControl properties
    QTest::addColumn<bool>("expectedCapturesVideo");
    QTest::addColumn<bool>("expectedCapturesPhotos");
    QTest::addColumn<bool>("expectedHasModes");
    QTest::addColumn<bool>("expectedHasZoom");
    QTest::addColumn<bool>("expectedHasVideoStream");
    QTest::addColumn<bool>("expectedPhotosInVideoMode");
    QTest::addColumn<bool>("expectedVideoInPhotoMode");
    QTest::addColumn<bool>("expectedHasTracking");

    // Note: xCapVid and xCapImg (capturesVideo/capturesPhotos) are "virtual" capabilities that account for
    // local gstreamer support. When a camera has a video stream (HAS_VIDEO_STREAM), QGC can locally record
    // video via gstreamer and screen-grab photos from the stream, even if the camera itself doesn't support
    // native video recording or photo capture. See "video-stream-only" and "stream-no-native-capture" rows.

    //                                       capVid   capImg   modes    stream   imgInVid vidInImg zoom     trkPt    trkRect   xCapVid  xCapImg  xModes   xZoom    xStream  xImgVid  xVidImg  xTrack
    QTest::newRow("all-caps")                << true  << true  << true  << true  << true  << true  << true  << true  << true   << true  << true  << true  << true  << true  << true  << true  << true;
    QTest::newRow("no-caps")                 << false << false << false << false << false << false << false << false << false  << false << false << false << false << false << false << false << false;
    QTest::newRow("photo-only")              << false << true  << false << false << false << false << false << false << false  << false << true  << false << false << false << false << false << false;
    QTest::newRow("video-only")              << true  << false << false << false << false << false << false << false << false  << true  << false << false << false << false << false << false << false;
    QTest::newRow("video-stream-only")       << false << false << false << true  << false << false << false << false << false  << true  << true  << false << false << true  << false << false << false;
    QTest::newRow("modes-zoom")              << false << true  << true  << false << false << false << true  << false << false  << false << true  << true  << true  << false << false << false << false;
    QTest::newRow("tracking-point")          << false << true  << false << false << false << false << false << true  << false  << false << true  << false << false << false << false << false << true;
    QTest::newRow("tracking-rect")           << false << true  << false << false << false << false << false << false << true   << false << true  << false << false << false << false << false << true;
    QTest::newRow("tracking-both")           << false << true  << false << false << false << false << false << true  << true   << false << true  << false << false << false << false << false << true;
    QTest::newRow("image-in-video")          << true  << true  << true  << false << true  << false << false << false << false  << true  << true  << true  << false << false << true  << false << false;
    QTest::newRow("video-in-image")          << true  << true  << true  << false << false << true  << false << false << false  << true  << true  << true  << false << false << false << true  << false;
    QTest::newRow("stream-no-native-capture")<< false << false << false << true  << false << false << false << false << false  << true  << true  << false << false << true  << false << false << false;
    QTest::newRow("stream-plus-photo")       << false << true  << false << true  << false << false << false << false << false  << true  << true  << false << false << true  << false << false << false;
}

void VehicleCameraControlTest::_testCameraCapFlags()
{
    // Fetch test data
    QFETCH(bool, captureVideo);
    QFETCH(bool, captureImage);
    QFETCH(bool, hasModes);
    QFETCH(bool, hasVideoStream);
    QFETCH(bool, canCaptureImageInVideoMode);
    QFETCH(bool, canCaptureVideoInImageMode);
    QFETCH(bool, hasBasicZoom);
    QFETCH(bool, hasTrackingPoint);
    QFETCH(bool, hasTrackingRectangle);

    QFETCH(bool, expectedCapturesVideo);
    QFETCH(bool, expectedCapturesPhotos);
    QFETCH(bool, expectedHasModes);
    QFETCH(bool, expectedHasZoom);
    QFETCH(bool, expectedHasVideoStream);
    QFETCH(bool, expectedPhotosInVideoMode);
    QFETCH(bool, expectedVideoInPhotoMode);
    QFETCH(bool, expectedHasTracking);

    // Create MockConfiguration with camera enabled and custom flags
    auto* mockConfig = newMockConfiguration(QStringLiteral("CameraCapFlagsTest"));
    mockConfig->setEnableCamera(true);
    mockConfig->setCameraCaptureVideo(captureVideo);
    mockConfig->setCameraCaptureImage(captureImage);
    mockConfig->setCameraHasModes(hasModes);
    mockConfig->setCameraHasVideoStream(hasVideoStream);
    mockConfig->setCameraCanCaptureImageInVideoMode(canCaptureImageInVideoMode);
    mockConfig->setCameraCanCaptureVideoInImageMode(canCaptureVideoInImageMode);
    mockConfig->setCameraHasBasicZoom(hasBasicZoom);
    mockConfig->setCameraHasTrackingPoint(hasTrackingPoint);
    mockConfig->setCameraHasTrackingRectangle(hasTrackingRectangle);

    _connectVehicle(mockConfig);
    if (QTest::currentTestFailed()) {
        return;
    }

    // Camera 1 has our configured flags. Camera 2 is always photo-only and not what we're testing.
    MavlinkCameraControlInterface* const camera = _firstCamera();
    QVERIFY2(camera, "Camera 1 (MAV_COMP_ID_CAMERA) not found in camera list");

    // Verify capability properties match expected values
    QCOMPARE(camera->capturesVideo(),     expectedCapturesVideo);
    QCOMPARE(camera->capturesPhotos(),    expectedCapturesPhotos);
    QCOMPARE(camera->hasModes(),          expectedHasModes);
    QCOMPARE(camera->hasZoom(),           expectedHasZoom);
    QCOMPARE(camera->hasVideoStream(),    expectedHasVideoStream);
    QCOMPARE(camera->photosInVideoMode(), expectedPhotosInVideoMode);
    QCOMPARE(camera->videoInPhotoMode(),  expectedVideoInPhotoMode);
    QCOMPARE(camera->hasTracking(),       expectedHasTracking);

    // hasFocus is always false since MockLinkCamera doesn't support CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS
    QCOMPARE(camera->hasFocus(), false);
}

void VehicleCameraControlTest::_testZoomTriggersCameraSettingsRequest()
{
    // A spec-minimal camera does not broadcast CAMERA_SETTINGS after a zoom change,
    // so QGC must re-request it itself once a zoom command is accepted. Otherwise
    // the zoom level (and FOV) shown by QGC goes stale after the first zoom.

    auto* mockConfig = newMockConfiguration(QStringLiteral("CameraZoomSettingsTest"));
    mockConfig->setEnableCamera(true);
    mockConfig->setCameraCaptureImage(true);
    mockConfig->setCameraHasBasicZoom(true);

    _connectVehicle(mockConfig);
    if (QTest::currentTestFailed()) {
        return;
    }

    MavlinkCameraControlInterface* const camera = _firstCamera();
    QVERIFY2(camera, "Camera 1 (MAV_COMP_ID_CAMERA) not found in camera list");
    QVERIFY(camera->hasZoom());

    // Wait for the initial CAMERA_SETTINGS exchange to settle: the mock reports zoom
    // level 1.0, so once QGC shows it the initial request/retry cycle is complete.
    QVERIFY_TRUE_WAIT(qFuzzyCompare(camera->zoomLevel(), 1.0), TestTimeout::longMs());

    // QGC alternates between REQUEST_MESSAGE and the deprecated REQUEST_CAMERA_SETTINGS
    // on retries (dual-transport migration), so accept either as a settings request.
    auto settingsRequestCount = [this]() {
        return _mockLink->receivedRequestMessageCount(MAV_COMP_ID_CAMERA, MAVLINK_MSG_ID_CAMERA_SETTINGS)
             + _mockLink->receivedMavCommandCount(MAV_CMD_REQUEST_CAMERA_SETTINGS, MAV_COMP_ID_CAMERA);
    };

    // An accepted zoom command must cause QGC to re-request CAMERA_SETTINGS on its own
    const int baselineAfterConnect = settingsRequestCount();
    camera->setZoomLevel(50.0);
    QTRY_VERIFY2_WITH_TIMEOUT(settingsRequestCount() > baselineAfterConnect,
                              "QGC did not re-request CAMERA_SETTINGS after setZoomLevel was accepted",
                              TestTimeout::longMs());
    // And the fresh settings update QGC's stale zoom level
    QVERIFY_TRUE_WAIT(qFuzzyCompare(camera->zoomLevel(), 50.0), TestTimeout::longMs());

    // Continuous zoom (start/stop) must trigger a refresh as well
    const int baselineAfterSetZoom = settingsRequestCount();
    camera->startZoom(1);
    camera->stopZoom();
    QTRY_VERIFY2_WITH_TIMEOUT(settingsRequestCount() > baselineAfterSetZoom,
                              "QGC did not re-request CAMERA_SETTINGS after startZoom/stopZoom were accepted",
                              TestTimeout::longMs());
}

void VehicleCameraControlTest::_testExtraStreamsStartAndStop()
{
    auto* mockConfig = newMockConfiguration(QStringLiteral("CameraExtraStreamsTest"));
    mockConfig->setEnableCamera(true);
    mockConfig->setCameraHasVideoStream(true);

    _connectVehicle(mockConfig);
    if (QTest::currentTestFailed()) {
        return;
    }

    MavlinkCameraControlInterface* const camera = _firstCamera();
    QVERIFY2(camera, "Camera 1 (MAV_COMP_ID_CAMERA) not found in camera list");
    QGCCameraManager* const cameraManager = _vehicle->cameraManager();
    // The mock camera advertises its streams 1 and 2.
    QVERIFY_TRUE_WAIT(camera->streamLabels().size() == 2, TestTimeout::longMs());
    QCOMPARE(camera->currentStream(), 0);

    const auto transmitted = [this](uint8_t streamId) {
        return _mockLink->cameraStreamTransmitted(MAV_COMP_ID_CAMERA, streamId);
    };
    const auto started = [this] {
        return _mockLink->receivedMavCommandCount(MAV_CMD_VIDEO_START_STREAMING, MAV_COMP_ID_CAMERA);
    };
    const auto stopped = [this] {
        return _mockLink->receivedMavCommandCount(MAV_CMD_VIDEO_STOP_STREAMING, MAV_COMP_ID_CAMERA);
    };
    // QGC requests a stream's status with REQUEST_MESSAGE or the deprecated command, in turn.
    const auto statusRequests = [this] {
        return _mockLink->receivedRequestMessageCount(MAV_COMP_ID_CAMERA, MAVLINK_MSG_ID_VIDEO_STREAM_STATUS) +
               _mockLink->receivedMavCommandCount(MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, MAV_COMP_ID_CAMERA);
    };

    // A camera transmits its first stream, the current one, without being asked.
    QCOMPARE(started(), 0);
    QVERIFY(transmitted(1));
    QVERIFY(!transmitted(2));

    // A stream shown besides the current one is started.
    camera->setExtraStreams({2});
    QVERIFY_TRUE_WAIT(transmitted(2), TestTimeout::mediumMs());
    QCOMPARE(started(), 1);
    QCOMPARE(stopped(), 0);

    // It becomes the current stream while the former one stays on display, as whoever displays the streams says when
    // told of the change: the camera is asked to stop or start neither. The request for the new current stream's
    // status follows any such command.
    const QMetaObject::Connection display =
        connect(cameraManager, &QGCCameraManager::streamChanged, camera, [camera] { camera->setExtraStreams({1}); });
    const int statusRequestsBefore = statusRequests();
    camera->setCurrentStream(1);
    QVERIFY_TRUE_WAIT(statusRequests() > statusRequestsBefore, TestTimeout::mediumMs());
    QVERIFY(disconnect(display));
    QCOMPARE(started(), 1);
    QCOMPARE(stopped(), 0);
    QVERIFY(transmitted(1));
    QVERIFY(transmitted(2));

    // The former stream leaves the display: it is stopped.
    camera->setExtraStreams({});
    QVERIFY_TRUE_WAIT(!transmitted(1), TestTimeout::mediumMs());
    QVERIFY(transmitted(2));
    QCOMPARE(started(), 1);

    // With no other stream on display, a change of the current stream stops the old one and starts the new one.
    camera->setCurrentStream(0);
    QVERIFY_TRUE_WAIT(transmitted(1) && !transmitted(2), TestTimeout::mediumMs());
    QCOMPARE(started(), 2);
    QCOMPARE(stopped(), 2);

    // Leaving the vehicle stops the extra stream and then the current one, and coming back to it starts them again:
    // each time two commands of one kind. The vehicle refuses a command while the camera has yet to answer the same
    // one, so the second waits for the first's answer.
    camera->setExtraStreams({2});
    QVERIFY_TRUE_WAIT(transmitted(2), TestTimeout::mediumMs());
    camera->setExtraStreams({});
    camera->stopStream();
    QVERIFY_TRUE_WAIT(!transmitted(1) && !transmitted(2), TestTimeout::mediumMs());
    camera->resumeStream();
    camera->setExtraStreams({2});
    QVERIFY_TRUE_WAIT(transmitted(1) && transmitted(2), TestTimeout::mediumMs());
    QCOMPARE(started(), 5);
    QCOMPARE(stopped(), 4);
}

UT_REGISTER_TEST(VehicleCameraControlTest, TestLabel::Integration, TestLabel::Vehicle)
