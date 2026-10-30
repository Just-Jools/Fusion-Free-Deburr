"""Floating HTML palette for picking a ball endmill. The command dialog stays
open while it's shown; the chosen entry id is passed back via a callback."""

import json
import os
import threading
import traceback

import adsk.core

PALETTE_ID = 'additionalToolpaths_toolPicker'
CLOUD_EVENT_ID = 'additionalToolpaths_loadCloudTools'
WIDTH, HEIGHT = 820, 520
HTML_PATH =os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', 'tool_picker', 'index.html')

_app: adsk.core.Application = None
_ui: adsk.core.UserInterface = None
_log = print
_handlers = []

_palette: adsk.core.Palette = None
_page_ready = False
_catalog = None
_units: adsk.core.UnitsManager = None
_tool_types = None
_on_pick = None


def start(app, ui, log, handlers):
    global _app, _ui, _log, _handlers
    _app, _ui, _log, _handlers = app, ui, log, handlers
    event = _app.registerCustomEvent(CLOUD_EVENT_ID)
    handler = _LoadCloudHandler()
    event.add(handler)
    _handlers.append(handler)


def stop():
    global _palette, _page_ready
    palette = _ui.palettes.itemById(PALETTE_ID) if _ui else None
    if palette:
        palette.deleteMe()
    _palette, _page_ready = None, False
    if _app:
        _app.unregisterCustomEvent(CLOUD_EVENT_ID)


def open_picker(catalog, units: adsk.core.UnitsManager, on_pick, tool_types=None):
    """Show the picker. on_pick(entry_id) is called with the chosen catalog entry.
    tool_types limits which tool types are offered (None = all)."""
    global _palette, _page_ready, _catalog, _units, _tool_types, _on_pick
    _catalog, _units, _tool_types, _on_pick = catalog, units, tool_types, on_pick

    _palette = _ui.palettes.itemById(PALETTE_ID)
    if not _palette:
        _page_ready = False
        # Create it visible: a palette created hidden and shown later can leave
        # the embedded browser stuck at a tiny size.
        _palette = _ui.palettes.add(PALETTE_ID, 'Select Ball Endmill', HTML_PATH.replace('\\', '/'),
                                    True, True, True, WIDTH, HEIGHT)
        handler = _IncomingHandler()
        _palette.incomingFromHTML.add(handler)
        _handlers.append(handler)
    else:
        if _page_ready:
            _palette.sendInfoToHTML('init', _payload())
        _palette.isVisible = True
    # Re-applying the size after showing forces the browser to lay out to fit.
    _palette.setMinimumSize(560, 320)
    _palette.setSize(WIDTH, HEIGHT)
    _center_on_viewport()


def _center_on_viewport():
    """Center the palette over Fusion's graphics area (so it opens on Fusion's monitor)."""
    viewport = _app.activeViewport
    if not viewport:
        return
    center = viewport.viewToScreen(adsk.core.Point2D.create(viewport.width / 2, viewport.height / 2))
    if not center:
        return
    left, top = int(center.x - WIDTH / 2), int(center.y - HEIGHT / 2)
    _palette.setPosition(left, top)
    _log(f'Tool picker centered at viewport center ({center.x:.0f}, {center.y:.0f}) -> ({left}, {top})')


def close_picker():
    global _on_pick
    _on_pick = None
    if _palette and _palette.isValid:
        _palette.isVisible = False


def _payload():
    def fmt(cm):
        return _units.formatValue(cm) if cm is not None else ''

    entries = [dict(e, diameter=fmt(e['diameterCm']), flute=fmt(e['fluteCm']))
               for e in _catalog.entries
               if _tool_types is None or e['type'] in _tool_types]
    unit = _units.defaultLengthUnits
    return json.dumps({
        'entries': entries,
        'cloudLoaded': _catalog.cloud_loaded,
        # Filter inputs are typed in document units; JS converts with cmPerUnit.
        'unit': {'label': unit, 'cmPerUnit': _units.convert(1, unit, 'cm')},
    })


class _IncomingHandler(adsk.core.HTMLEventHandler):
    def notify(self, args: adsk.core.HTMLEventArgs):
        global _page_ready
        try:
            action = args.action
            data = json.loads(args.data) if args.data else {}
            if action == 'ready':
                _page_ready = True
                args.returnData = _payload()
            elif action == 'select':
                callback = _on_pick
                close_picker()
                if callback:
                    callback(data['id'])
            elif action == 'cancel':
                close_picker()
            elif action == 'loadCloud':
                # Return now so the page can show its spinner, then do the slow
                # query on Fusion's main thread via a custom event.
                threading.Timer(0.1, lambda: _app.fireCustomEvent(CLOUD_EVENT_ID)).start()
                args.returnData = 'ok'
        except:
            _ui.messageBox(f'Tool picker failed:\n{traceback.format_exc()}')


class _LoadCloudHandler(adsk.core.CustomEventHandler):
    def notify(self, args):
        try:
            if _catalog and not _catalog.cloud_loaded:
                _catalog.load_cloud()
            if _palette and _palette.isValid:
                _palette.sendInfoToHTML('init', _payload())
        except Exception as e:
            _log(f'Cloud tool load failed: {traceback.format_exc()}')
            if _palette and _palette.isValid:
                _palette.sendInfoToHTML('error', f'Could not load cloud libraries: {e}')
