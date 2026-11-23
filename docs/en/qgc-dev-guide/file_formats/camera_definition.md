# Camera Definition Extensions

Cameras that support the [MAVLink Camera Protocol](https://mavlink.io/en/services/camera.html) describe their settings in a [MAVLink Camera Definition File](https://mavlink.io/en/services/camera_def.html).
_QGroundControl_ builds the camera settings page (the gear icon of the camera tools) from that file.

In addition to the elements defined by the MAVLink specification, _QGroundControl_ understands the elements and attributes described on this page.
They let a camera add buttons, explanatory text and visual groups to its settings page.

## Compatibility

All the elements on this page are placed inside the `<parameters>` element, where they appear in the settings page in the order in which they are defined:

```xml
<parameters>
    <parameter name="CAM_MODE" ...>...</parameter>
    <text>...</text>
    <action name="CAM_FORMAT" ...>...</action>
    <group name="GRP_EXPOSURE" ...>
        <parameter name="CAM_ISO" ...>...</parameter>
    </group>
</parameters>
```

Versions of _QGroundControl_ that don't support them ignore these elements, and show the parameters inside a `<group>` like any other parameter.
A camera can therefore use a single definition file for all versions of _QGroundControl_.

::: warning
Don't add a second `<parameters>` element: _QGroundControl_ only reads the first one.
:::

::: tip
_QGroundControl_ caches definition files by vendor, model and `cam_definition_version` (from `CAMERA_INFORMATION`).
Increase `cam_definition_version` whenever the definition file changes, or _QGroundControl_ keeps using the cached copy.
:::

Actions, texts and groups each have a name, which must not be used by any parameter, action, text or group of the same definition.
Like a parameter, they can be hidden with an [exclusion](https://mavlink.io/en/services/camera_def.html#exclusion-rules) by naming them in an `<exclude>` element.

## Actions

An action shows one or more buttons.
Each click of a button sends the button's value to the camera with `PARAM_EXT_SET`, using the action's name as the parameter id, even when the same value was sent before.

```xml
<action name="CAM_FORMAT" type="uint8">
    <description>Format SD Card</description>
    <buttons>
        <button value="1" label="Format">
            <confirm title="Format SD Card" yes="Format" no="Cancel">All files on the SD card will be erased. Continue?</confirm>
        </button>
    </buttons>
</action>
```

| Element / attribute | Description |
| --- | --- |
| `action` `name` | Parameter id sent to the camera (up to 16 characters). |
| `action` `type` | Type of the value sent, one of the integer types, `bool`, `float` or `double`. |
| `action` `highlight` | `1` to mark the button whose value the camera reports for the action. Default `0`. |
| `description` | Label of the action in the settings page. |
| `button` `value` | Value sent by the button. It must be valid for the action's `type`. |
| `button` `label` | Text of the button. |
| `confirm` | Optional. The user has to confirm its text in a dialog before the value is sent. |
| `confirm` `title` | Title of the dialog. Default: the action's description. |
| `confirm` `yes`, `no` | Texts of the dialog's buttons. Default: _Yes_ and _No_. |

The camera answers with `PARAM_EXT_ACK`:

- `PARAM_ACK_ACCEPTED` completes the action.
- `PARAM_ACK_IN_PROGRESS` keeps _QGroundControl_ waiting; send it again within 3 seconds for an action that takes longer.
- `PARAM_ACK_FAILED` or `PARAM_ACK_VALUE_UNSUPPORTED` is reported to the user as a failure, as is no answer within 3 seconds.

An action is never sent again by _QGroundControl_ on its own, so that it can't run twice.
Its buttons are disabled while _QGroundControl_ waits for the answer.

If `highlight` is set, _QGroundControl_ reads the action's value from the camera like a parameter value, so the camera must report it with `PARAM_EXT_VALUE`.

## Texts

A text shows a line of text across the full width of the settings page.

```xml
<text name="TXT_PHOTO_HINT" size="small" bold="1" align="center">Resolution and quality apply to JPEG images only.</text>
```

| Attribute | Description |
| --- | --- |
| `name` | Optional. Only needed to exclude the text. |
| `size` | `default`, `small` or `large`. Default `default`. |
| `bold` | `1` for bold text. Default `0`. |
| `align` | `left`, `center` or `right`. Default `left`. |

White space around the text is removed. Use `&#xA;` for a line break inside the text.

## Groups

A group shows the parameters, actions and texts inside it on a common background, under an optional title.
Excluding the group hides everything inside it.

```xml
<group name="GRP_PHOTO" title="Photo" color="#4000A0FF" radius="6">
    <parameter name="CAM_PHOTORES" type="uint32" default="0">...</parameter>
    <parameter name="CAM_PHOTOQUAL" type="uint32" default="1">...</parameter>
</group>
```

| Attribute | Description |
| --- | --- |
| `name` | Optional. Only needed to exclude the group. |
| `title` | Optional bold title at the top of the group. A text inside the group can be used instead. |
| `color` | Optional background color as `#RRGGBB` or `#AARRGGBB` (alpha first). Default: the theme's color. |
| `radius` | Optional corner radius of the background, in pixels. Default: the theme's radius. |

Groups can't be nested.
A group with nothing left to show, because everything in it is excluded, isn't shown either.

::: info
Versions of _QGroundControl_ that don't support groups ignore an exclusion of the group's name.
To hide the parameters of a group in these versions too, exclude each of them as well.
:::

## Localization

The texts of actions, texts and groups are translated with the [localization](https://mavlink.io/en/services/camera_def.html#localization) section of the definition file, like any other text in it.

## Example

The [example camera definition](https://github.com/mavlink/qgroundcontrol/blob/master/src/Camera/camera_definition_example.xml) in the _QGroundControl_ source uses all the elements on this page.
