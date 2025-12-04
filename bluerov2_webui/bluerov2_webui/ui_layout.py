from nicegui import ui
import uuid
import asyncio
import json
from numbers import Number
from datetime import datetime

# ==========================================
# HELPER: FLATTEN DICTIONARY
# ==========================================
def flatten_dict(d, parent_key='', sep='.'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

# ==========================================
# COMPONENT: ECHO TAB (Reusable)
# ==========================================
class EchoTab:
    def __init__(self, rov_node, tabs_container, panels_container):
        self.rov_node = rov_node
        self.id = uuid.uuid4().hex
        self.active_keys = set() 
        self.latest_full_msg = None 
        
        # 1. Tab Header
        with tabs_container:
            self.tab_header = ui.tab(name=self.id, label='Echo') \
                .classes('h-full text-xs min-w-[60px] mx-1 rounded-t border-t border-l border-r border-transparent hover:border-slate-700')
        
        # 2. Tab Panel
        with panels_container:
            self.panel = ui.tab_panel(name=self.id).classes('w-full h-full p-0 flex flex-col bg-slate-900 overflow-hidden min-h-0 min-w-0')
            with self.panel:
                self._build_ui()
                
        tabs_container.set_value(self.id)
        ui.timer(0.0, self._refresh_topics, once=True)

    def _build_ui(self):
        # --- Toolbar ---
        with ui.row().classes('w-full items-center gap-1 p-2 bg-slate-800 border-b border-slate-700 shrink-0'):
            self.topic_select = ui.select([], label='Topic', with_input=True) \
                .classes('flex-grow text-xs min-w-[80px]').props('dense options-dense')
            self.topic_select.on_value_change(self._on_topic_change)
            
            # Table Mode Switch
            self.view_mode = ui.switch('Table').props('dense color=green size=xs').classes('text-xs text-slate-400 mr-2')
            
            ui.button(icon='refresh', on_click=self._refresh_topics).props('flat dense size=sm').classes('text-slate-400')
            ui.button(icon='settings', on_click=self._open_filter_dialog).props('flat dense size=sm').classes('text-slate-400')
            ui.button(icon='close', on_click=self.close).props('flat dense size=sm').classes('text-red-400')

        # --- CONTENT STAGE (The Fix) ---
        # We create a relative container that fills the remaining space.
        # The Text and Table will be ABSOLUTE positioned inside this.
        # This prevents the "Resize needed" bug because the container size is fixed by the parent column.
        with ui.element('div').classes('w-full flex-grow relative overflow-hidden min-h-0 min-w-0'):
            
            # --- VIEW 1: TEXT MODE ---
            # absolute inset-0 means "stick to all 4 edges"
            self.text_scroll = ui.scroll_area().classes('absolute top-0 left-0 w-full h-full bg-black/30 p-2')
            with self.text_scroll:
                self.content_label = ui.label('(Select a topic to echo)') \
                    .classes('text-xs font-mono text-slate-500 whitespace-pre-wrap break-all w-full')
            
            # --- VIEW 2: TABLE MODE ---
            # absolute inset-0
            self.table_container = ui.element('div').classes('absolute top-0 left-0 w-full h-full bg-slate-800 z-10')
            with self.table_container:
                columns = [
                    {'name': 'key', 'label': 'Field', 'field': 'key', 'align': 'left', 'classes': 'break-all whitespace-pre-wrap text-[10px] font-mono text-slate-400', 'style': 'width: 40%'},
                    {'name': 'val', 'label': 'Value', 'field': 'val', 'align': 'left', 'classes': 'break-all whitespace-pre-wrap text-[10px] font-mono text-green-400', 'style': 'width: 60%'},
                ]
                self.content_table = ui.table(columns=columns, rows=[], row_key='key') \
                    .classes('w-full h-full border-none') \
                    .props('dense flat square hide-bottom wrap-cells')
            
            # Bind Visibility
            # When Table is ON, Table Container is visible.
            self.table_container.bind_visibility_from(self.view_mode, 'value')
            # We don't need to hide the text scroll strictly (z-index handles it), 
            # but hiding it improves performance.
            self.text_scroll.bind_visibility_from(self.view_mode, 'value', value=False)

    def _open_filter_dialog(self):
        if not self.latest_full_msg:
            ui.notify('Wait for a message to arrive before filtering', type='warning')
            return

        flat_data = flatten_dict(self.latest_full_msg)
        all_keys = sorted(flat_data.keys())

        with ui.dialog().classes('backdrop-blur-sm bg-black/60') as dialog:
            with ui.card().classes('bg-slate-800 text-white w-[600px] h-[600px] max-h-[90vh] flex flex-col p-0'):
                
                with ui.row().classes('w-full items-center justify-between p-4 shrink-0 border-b border-slate-700'):
                    ui.label('Select Fields to Echo').classes('font-bold text-lg')
                    ui.icon('filter_list', size='sm').classes('text-slate-400')

                with ui.scroll_area().classes('flex-grow w-full p-4 bg-slate-900/50'):
                    self.checkboxes = {}
                    for key in all_keys:
                        is_checked = key in self.active_keys
                        with ui.row().classes('w-full items-center justify-between hover:bg-slate-700/50 rounded px-2 py-1 mb-1'):
                            ui.label(key).classes('text-xs text-slate-300 font-mono break-all mr-2')
                            self.checkboxes[key] = ui.checkbox(value=is_checked).props('dense size=xs color=green')

                with ui.row().classes('w-full justify-between p-4 shrink-0 border-t border-slate-700 bg-slate-800'):
                    def clear_filter():
                        self.active_keys = set()
                        dialog.close()
                        ui.notify('Filter cleared (Showing all)')

                    def apply_filter():
                        self.active_keys = {k for k, cb in self.checkboxes.items() if cb.value}
                        dialog.close()
                        ui.notify(f'Filter applied: {len(self.active_keys)} fields')

                    ui.button('Reset', on_click=clear_filter).props('flat color=red dense')
                    ui.button('Apply Filter', on_click=apply_filter).props('color=green dense unelevated').classes('px-6')
                
        dialog.open()

    async def _refresh_topics(self):
        topics = self.rov_node.get_all_topics()
        options = []
        ignored_types = ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage']
        for t in topics:
            if t[1][0] not in ignored_types:
                options.append(f"{t[0]} | {t[1][0]}")
        self.topic_select.options = sorted(options)
        self.topic_select.update()

    def _on_topic_change(self, e):
        if not e.value: 
            self.content_label.text = "(Select a topic to echo)"
            self.content_label.classes(replace='text-slate-500')
            self.content_table.rows = []
            return
            
        self.active_keys = set()
        self.latest_full_msg = None
        
        parts = e.value.split('|')
        topic = parts[0].strip()
        type_str = parts[1].strip()
        
        self.tab_header._props['label'] = topic.split('/')[-1]
        self.tab_header.update()
        
        self.content_label.text = "Waiting for data..."
        self.content_label.classes(replace='text-green-400')
        
        self.rov_node.subscribe_dynamic(topic, type_str, self._on_msg)

    def _on_msg(self, msg_dict):
        self.latest_full_msg = msg_dict
        
        if self.active_keys:
            flat = flatten_dict(msg_dict)
            display_data = {k: v for k, v in flat.items() if k in self.active_keys}
        else:
            display_data = msg_dict

        if self.view_mode.value:
            # --- TABLE MODE ---
            if not self.active_keys:
                 flat_table_data = flatten_dict(msg_dict)
            else:
                 flat_table_data = display_data
            
            rows = [{'key': k, 'val': str(v)} for k, v in flat_table_data.items()]
            rows.sort(key=lambda x: x['key'])
            self.content_table.rows = rows
            self.content_table.update()
        else:
            # --- TEXT MODE ---
            try:
                text = json.dumps(display_data, indent=2, default=str)
                if len(text) > 10000: text = text[:10000] + "\n... [TRUNCATED]"
                self.content_label.text = text
            except: pass

    def close(self):
        try: self.panel.delete(); self.tab_header.delete()
        except: pass


# ==========================================
# COMPONENT: NODE INSPECTOR (Right Panel)
# ==========================================
class NodeInspectorTab:
    def __init__(self, rov_node, tabs_container, panels_container):
        self.rov_node = rov_node
        self.tabs_container = tabs_container
        self.panels_container = panels_container
        self.id = uuid.uuid4().hex
        
        with self.tabs_container:
            self.tab_header = ui.tab(name=self.id, label='New Tab') \
                .classes('h-full text-xs min-w-[80px] mx-1 rounded-t hover:bg-slate-800')
        
        with self.panels_container:
            self.panel = ui.tab_panel(name=self.id).classes('w-full h-full p-0 flex flex-col min-w-0')
            with self.panel: self.build_content()
            
        self.tabs_container.set_value(self.id)
        ui.timer(0.0, self.refresh_list, once=True)

    def build_content(self):
        with ui.row().classes('w-full items-center gap-1 p-2 border-b border-slate-700 bg-slate-800'):
            self.node_selector = ui.select([], label='Select Node', with_input=True).classes('flex-grow text-xs').on_value_change(self.on_node_selected)
            ui.button(icon='refresh', on_click=self.refresh_list).props('flat dense size=sm').classes('text-slate-400')
            ui.button(icon='close', on_click=self.close).props('flat dense size=sm').classes('text-red-400 hover:bg-slate-700')
        self.param_area = ui.scroll_area().classes('w-full flex-grow p-2 bg-slate-900/30')

    def close(self):
        if self.tabs_container.value == self.id: self.tabs_container.set_value(None)
        try: self.panel.delete(); self.tab_header.delete()
        except: pass

    async def refresh_list(self):
        nodes = self.rov_node.get_active_nodes()
        node_names = [f"{ns}/{name}".replace('//', '/') for name, ns in nodes]
        node_names.sort()
        self.node_selector.options = node_names
        self.node_selector.update()

    async def on_node_selected(self, e):
        selected_node = e.value
        if not selected_node: return
        short_name = selected_node.split('/')[-1]
        self.tab_header._props['label'] = short_name
        self.tab_header.update()
        self.param_area.clear()
        ui.notify(f'Loading {short_name}...')
        params = await self.rov_node.get_node_parameters(selected_node.strip('/'))
        if params is None: ui.notify('Node not reachable', color='red'); return
        with self.param_area:
            ui.element('div').classes('h-2') 
            for name, value in params.items(): self._create_param_widget(selected_node, name, value)

    def _create_param_widget(self, node_name, param_name, value):
        with ui.card().classes('w-full bg-slate-700 p-2 mb-2 border-l-4 border-slate-500'):
            ui.label(param_name).classes('text-[10px] text-slate-400 font-mono break-all uppercase mb-1')
            if isinstance(value, bool): ui.switch(value=value, on_change=lambda e: self.rov_node.set_remote_parameter(node_name.strip('/'), param_name, e.value)).props('dense')
            elif isinstance(value, (int, float)): 
                step = 1 if isinstance(value, int) else 0.01
                ui.number(value=value, format='%.2f', step=step, on_change=lambda e: self.rov_node.set_remote_parameter(node_name.strip('/'), param_name, e.value)).classes('w-full bg-slate-800 rounded px-2 text-white text-sm')
            elif isinstance(value, str): ui.input(value=value, on_change=lambda e: self.rov_node.set_remote_parameter(node_name.strip('/'), param_name, e.value)).classes('w-full text-white text-sm')


# ==========================================
# MAIN UI BUILDER
# ==========================================
def build_interface(rov_node):
    @ui.page('/')
    def index():
        # --- HEADER ---
        with ui.header().classes('bg-slate-900 shadow-lg items-center gap-4'):
            ui.label('BlueROV2 Cockpit').classes('text-lg font-bold tracking-wide mr-4')
            
            def set_mode(e):
                rov_node.set_remote_parameter('bluerov2_teleop', 'mode', e.value)
                ui.notify(f'MODE: {e.value.upper()}', type='info')
            
            ui.select(['manual', 'servoing', 'correction', 'searching'], label='Teleop Mode', value='manual', on_change=set_mode) \
                .classes('w-32 bg-slate-700 rounded-md text-white')
            
            ui.space()
            ui.label('System: ONLINE').classes('text-green-400 text-xs font-mono')

        # --- LAYOUT (Responsive Fixes) ---
        with ui.row().classes('w-full h-[calc(100vh-60px)] no-wrap gap-0 overflow-x-auto'):
            
            # --- LEFT PANEL (Topic Inspector) ---
            with ui.column().classes('w-1/4 min-w-[300px] h-full bg-slate-800 border-r border-slate-700 p-0 overflow-hidden min-w-0'):
                
                with ui.splitter(horizontal=True, value=50).classes('w-full h-full') as splitter:
                    
                    # TOP ECHO WINDOW
                    with splitter.before:
                        with ui.column().classes('w-full h-full flex flex-col overflow-hidden min-h-[200px] min-w-0'):
                            with ui.row().classes('w-full h-8 bg-slate-900 items-center pl-2 pr-2 gap-2 no-wrap overflow-y-hidden shrink-0'):
                                with ui.row().classes('flex-grow h-full overflow-x-auto overflow-y-hidden no-wrap items-center pr-8'):
                                    with ui.tabs().classes('h-full text-xs text-slate-400') as echo_tabs_top: pass
                                ui.button(icon='add', on_click=lambda: EchoTab(rov_node, echo_tabs_top, echo_panels_top)).props('flat round dense').classes('text-green-400 shrink-0')
                            
                            with ui.tab_panels(echo_tabs_top, value=None).classes('w-full flex-grow bg-slate-800 overflow-hidden min-h-0 min-w-0') as echo_panels_top: pass
                            EchoTab(rov_node, echo_tabs_top, echo_panels_top)

                    # BOTTOM ECHO WINDOW
                    with splitter.after:
                        with ui.column().classes('w-full h-full flex flex-col border-t border-slate-700 overflow-hidden min-h-[200px] min-w-0'):
                            with ui.row().classes('w-full h-8 bg-slate-900 items-center pl-2 pr-2 gap-2 no-wrap overflow-y-hidden shrink-0'):
                                with ui.row().classes('flex-grow h-full overflow-x-auto overflow-y-hidden no-wrap items-center pr-8'):
                                    with ui.tabs().classes('h-full text-xs text-slate-400') as echo_tabs_bot: pass
                                ui.button(icon='add', on_click=lambda: EchoTab(rov_node, echo_tabs_bot, echo_panels_bot)).props('flat round dense').classes('text-green-400 shrink-0')
                            
                            with ui.tab_panels(echo_tabs_bot, value=None).classes('w-full flex-grow bg-slate-800 overflow-hidden min-h-0 min-w-0') as echo_panels_bot: pass
                            EchoTab(rov_node, echo_tabs_bot, echo_panels_bot)

            # --- CENTER PANEL (Video) ---
            with ui.column().classes('flex-grow min-w-[400px] h-full p-4 items-center bg-black min-w-0'):
                video_source = ui.toggle({'raw': 'Raw', 'proc': 'Detect'}, value='proc').classes('bg-slate-700 text-white')
                with ui.card().classes('w-full p-0 border-2 border-slate-600'):
                    video_feed = ui.interactive_image().classes('w-full')
                def update_frame():
                    img = rov_node.latest_image_raw if video_source.value == 'raw' else rov_node.latest_image_processed
                    if img: video_feed.set_source(img)
                ui.timer(0.033, update_frame)

            # --- RIGHT PANEL (Node Params) ---
            with ui.column().classes('w-1/4 min-w-[300px] h-full border-l border-slate-700 bg-slate-800 flex flex-col min-w-0'):
                with ui.row().classes('w-full h-12 bg-slate-900 items-center pl-2 pr-2 gap-2 no-wrap overflow-y-hidden shrink-0'):
                    with ui.row().classes('flex-grow h-full overflow-x-auto overflow-y-hidden no-wrap items-center pr-8'):
                        with ui.tabs().classes('h-full text-xs text-slate-400') as r_tabs: pass 
                    ui.button(icon='add', on_click=lambda: NodeInspectorTab(rov_node, r_tabs, r_panels)).props('flat round dense').classes('text-green-400 shrink-0')
                with ui.tab_panels(r_tabs, value=None).classes('w-full flex-grow bg-slate-800 overflow-hidden min-h-0') as r_panels: pass
                NodeInspectorTab(rov_node, r_tabs, r_panels)