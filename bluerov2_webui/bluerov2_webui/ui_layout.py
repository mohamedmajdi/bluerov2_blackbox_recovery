from nicegui import ui, events
import uuid
import asyncio
import json
import yaml
from numbers import Number
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
# COMPONENT: ECHO TAB (Left Panel)
# ==========================================
class EchoTab:
    def __init__(self, rov_node, tabs_container, panels_container):
        self.rov_node = rov_node
        self.id = uuid.uuid4().hex
        self.active_keys = set() 
        self.latest_full_msg = None 
        self.current_topic_name = None 
        
        with tabs_container:
            self.tab_header = ui.tab(name=self.id, label='Echo') \
                .classes('h-8 text-[10px] min-w-[60px] border-r border-slate-800 bg-slate-900 text-slate-400 rounded-none') \
                .props('no-caps')
        
        with panels_container:
            self.panel = ui.tab_panel(name=self.id).classes('w-full h-full p-0 flex flex-col bg-black overflow-auto rounded-none')
            with self.panel:
                self._build_ui()
                
        tabs_container.set_value(self.id)
        ui.timer(0.0, self._refresh_topics, once=True)

    def _build_ui(self):
        with ui.row().classes('w-full items-center gap-1 p-1 bg-slate-900 border-b border-slate-800 shrink-0 overflow-x-auto no-wrap h-10'):
            self.topic_select = ui.select([], label='TOPIC', with_input=True) \
                .classes('flex-grow text-[10px] min-w-[120px] rounded-none').props('dense outlined square options-dense behavior="menu"')
            self.topic_select.on_value_change(self._on_topic_change)
            
            self.view_mode = ui.switch('TBL').props('dense color=cyan size=xs keep-color').classes('text-[10px] text-slate-500 mr-1 shrink-0')
            
            ui.button(icon='refresh', on_click=self._refresh_topics).props('flat dense square size=sm').classes('text-slate-400 hover:text-white shrink-0')
            ui.button(icon='filter_list', on_click=self._open_filter_dialog).props('flat dense square size=sm').classes('text-slate-400 hover:text-cyan-400 shrink-0')
            ui.button(icon='close', on_click=self.close).props('flat dense square size=sm').classes('text-red-500 hover:bg-red-900 shrink-0')

        self.text_scroll = ui.scroll_area().classes('w-full flex-grow bg-black p-2 min-h-[200px]')
        with self.text_scroll:
            self.content_label = ui.label('> SELECT TOPIC_') \
                .classes('text-[10px] font-mono text-green-500 whitespace-pre-wrap break-all w-full leading-tight')
        
        columns = [
            {'name': 'key', 'label': 'FIELD', 'field': 'key', 'align': 'left', 'classes': 'break-all whitespace-pre-wrap text-[10px] font-mono text-cyan-700 font-bold', 'style': 'width: 35%; border-bottom: 1px solid #222;'},
            {'name': 'val', 'label': 'VALUE', 'field': 'val', 'align': 'left', 'classes': 'break-all whitespace-pre-wrap text-[10px] font-mono text-green-500', 'style': 'width: 65%; border-bottom: 1px solid #222;'},
        ]
        
        with ui.column().classes('w-full flex-grow min-h-[200px] p-0 relative max-w-full overflow-hidden bg-black') as self.table_container:
            self.content_table = ui.table(columns=columns, rows=[], row_key='key') \
                .classes('w-full h-full bg-black border-none rounded-none') \
                .props('dense flat square hide-bottom wrap-cells virtual-scroll') 
            
        self.table_container.bind_visibility_from(self.view_mode, 'value')
        self.text_scroll.bind_visibility_from(self.view_mode, 'value', value=False)

    def _open_filter_dialog(self):
        if not self.latest_full_msg:
            ui.notify('NO DATA STREAM', type='warning', classes='rounded-none')
            return
        flat_data = flatten_dict(self.latest_full_msg)
        all_keys = sorted(flat_data.keys())
        
        with ui.dialog().classes('backdrop-blur-sm') as dialog:
            with ui.card().classes('bg-black border border-cyan-700 w-[500px] h-[600px] max-h-[90vh] flex flex-col p-0 rounded-none'):
                with ui.row().classes('w-full items-center justify-between p-2 shrink-0 border-b border-cyan-900 bg-cyan-950'):
                    ui.label('FILTER // DATA').classes('font-bold text-sm text-cyan-500 tracking-widest font-mono')
                    ui.icon('filter_alt', size='xs').classes('text-cyan-600')
                
                with ui.scroll_area().classes('flex-grow w-full p-2 bg-black'):
                    self.checkboxes = {}
                    for key in all_keys:
                        is_checked = key in self.active_keys
                        with ui.row().classes('w-full items-center justify-between hover:bg-slate-900 border-b border-slate-900 px-1 py-0'):
                            ui.label(key).classes('text-[10px] text-slate-400 font-mono break-all')
                            self.checkboxes[key] = ui.checkbox(value=is_checked).props('dense size=xs color=cyan keep-color')
                
                with ui.row().classes('w-full justify-between p-2 shrink-0 border-t border-slate-800 bg-black'):
                    def clear_filter(): self.active_keys = set(); dialog.close(); ui.notify('FILTER CLEARED', classes='rounded-none')
                    def apply_filter(): self.active_keys = {k for k, cb in self.checkboxes.items() if cb.value}; dialog.close(); ui.notify(f'FILTER APPLIED', classes='rounded-none')
                    
                    ui.button('RESET', on_click=clear_filter).props('flat dense square size=sm color=slate')
                    ui.button('APPLY', on_click=apply_filter).props('outline dense square size=sm color=cyan')
        dialog.open()

    async def _refresh_topics(self):
        topics = self.rov_node.get_all_topics()
        options = []
        ignored_types = ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage']
        for t in topics:
            if t[1][0] not in ignored_types: options.append(f"{t[0]} | {t[1][0]}")
        self.topic_select.options = sorted(options)
        self.topic_select.update()

    def _on_topic_change(self, e):
        if not e.value: 
            self.content_label.text = "> SELECT TOPIC_"
            self.content_label.classes(replace='text-slate-500')
            self.content_table.rows = []
            return
        
        if self.current_topic_name:
            if hasattr(self.rov_node, 'unsubscribe'):
                self.rov_node.unsubscribe(self.current_topic_name)
            
        self.latest_full_msg = None
        self.active_keys = set()
        self.content_table.rows = []
        self.content_label.text = "> ESTABLISHING LINK..."
        self.content_label.classes(replace='text-orange-500 animate-pulse')
        
        parts = e.value.split('|')
        topic = parts[0].strip(); type_str = parts[1].strip()
        
        self.current_topic_name = topic
        short_label = topic.split('/')[-1].upper()[:12]
        self.tab_header._props['label'] = short_label
        self.tab_header.update()
        
        self.rov_node.subscribe_dynamic(topic, type_str, self._on_msg)

    def _on_msg(self, msg_dict):
        self.latest_full_msg = msg_dict
        if self.active_keys:
            flat = flatten_dict(msg_dict)
            display_data = {k: v for k, v in flat.items() if k in self.active_keys}
        else:
            display_data = msg_dict

        if self.view_mode.value:
            if not self.active_keys: flat_table_data = flatten_dict(msg_dict)
            else: flat_table_data = display_data
            rows = [{'key': k, 'val': str(v)} for k, v in flat_table_data.items()]
            rows.sort(key=lambda x: x['key'])
            self.content_table.rows = rows
            self.content_table.update()
        else:
            try:
                text = json.dumps(display_data, indent=2, default=str)
                if len(text) > 10000: text = text[:10000] + "\n... [TRUNCATED]"
                self.content_label.text = text
                self.content_label.classes(replace='text-green-500')
            except: pass

    def close(self):
        if self.current_topic_name and hasattr(self.rov_node, 'unsubscribe'):
            self.rov_node.unsubscribe(self.current_topic_name)
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
        self.current_node_name = None
        self.local_params = {} 
        
        with self.tabs_container:
            self.tab_header = ui.tab(name=self.id, label='CONF') \
                .classes('h-8 text-[10px] min-w-[60px] border-l border-r border-slate-800 bg-slate-900 text-slate-400 rounded-none') \
                .props('no-caps')
        
        with self.panels_container:
            self.panel = ui.tab_panel(name=self.id).classes('w-full h-full min-h-[250px] p-0 flex flex-col min-w-0 bg-black overflow-auto rounded-none')
            with self.panel: self.build_content()
            
        self.tabs_container.set_value(self.id)
        ui.timer(0.0, self.refresh_list, once=True)

    def build_content(self):
        with ui.row().classes('w-full items-center gap-1 p-1 border-b border-slate-800 bg-slate-900 overflow-x-auto no-wrap h-10'):
            self.node_selector = ui.select([], label='NODE', with_input=True) \
                .classes('flex-grow text-[10px] min-w-[100px] rounded-none').props('dense outlined square options-dense').on_value_change(self.on_node_selected)
            
            ui.button(icon='file_upload', on_click=self.open_load_dialog).props('flat dense square size=sm').classes('text-yellow-600 hover:text-yellow-400 shrink-0').tooltip('Load')
            ui.button(icon='save', on_click=self.prompt_save_filename).props('flat dense square size=sm').classes('text-blue-600 hover:text-blue-400 shrink-0').tooltip('Save')
            ui.button(icon='refresh', on_click=self.refresh_list).props('flat dense square size=sm').classes('text-slate-500 hover:text-white shrink-0')
            ui.button(icon='close', on_click=self.close).props('flat dense square size=sm').classes('text-red-600 hover:text-red-400 shrink-0')

        with ui.row().classes('w-full p-1 bg-black border-b border-slate-800'):
            self.search_input = ui.input(placeholder='SEARCH_PARAM') \
                .props('dense outlined square input-class="text-[10px] text-green-400" placeholder-color="slate-700"') \
                .classes('w-full text-[10px] rounded-none').on_value_change(self.render_filtered_params)

        self.param_area = ui.scroll_area().classes('w-full flex-grow p-1 bg-black min-h-[200px]')

    # --- SAVE ---
    def prompt_save_filename(self):
        if not self.current_node_name or not self.local_params:
            ui.notify('NO PARAMS LOADED', type='warning', classes='rounded-none')
            return
        clean_name = self.current_node_name.strip('/').replace('/', '_')
        default_name = f"cfg_{clean_name}.yaml"
        with ui.dialog().classes('backdrop-blur-sm') as dialog:
            with ui.card().classes('bg-black border border-blue-600 w-96 p-4 rounded-none'):
                ui.label('EXPORT CONFIG').classes('font-bold text-lg text-blue-500 mb-2 tracking-widest font-mono')
                filename_input = ui.input('FILENAME', value=default_name).props('outlined dense square').classes('w-full mb-4 font-mono')
                def perform_save():
                    final_name = filename_input.value
                    if not final_name.endswith('.yaml'): final_name += '.yaml'
                    self.save_parameters_to_file(final_name); dialog.close()
                with ui.row().classes('w-full justify-end gap-2'):
                    ui.button('CANCEL', on_click=dialog.close).props('flat color=slate dense square')
                    ui.button('SAVE', on_click=perform_save).props('outline color=blue dense square')
        dialog.open()

    def save_parameters_to_file(self, filename):
        node_key = self.current_node_name
        if not node_key.startswith('/'): node_key = f'/{node_key}'
        yaml_structure = {node_key: {'ros__parameters': self.local_params}}
        try:
            yaml_content = yaml.dump(yaml_structure, default_flow_style=False)
            ui.download(yaml_content.encode('utf-8'), filename)
            ui.notify(f'SAVED: {filename}', type='positive', classes='rounded-none')
        except Exception as e: ui.notify(f'ERROR: {e}', type='negative', classes='rounded-none')

    # --- LOAD (PREVIEW) ---
    def open_load_dialog(self):
        if not self.current_node_name:
            ui.notify('SELECT NODE FIRST', type='warning', classes='rounded-none')
            return

        with ui.dialog().classes('backdrop-blur-sm') as dialog:
            self.dialog_card = ui.card().classes('bg-black border border-yellow-700 w-[600px] h-[500px] flex flex-col p-0 rounded-none')
            
            with self.dialog_card:
                with ui.column().classes('w-full h-full justify-center items-center p-8') as self.upload_state_container:
                    ui.icon('upload_file', size='xl').classes('text-slate-700 mb-4')
                    ui.label('UPLOAD CONFIG').classes('text-xl font-bold mb-2 text-slate-300 tracking-widest font-mono')
                    ui.label(f'TARGET: {self.current_node_name}').classes('text-xs text-yellow-600 mb-6 font-mono')
                    
                    ui.upload(on_upload=lambda e: self.process_uploaded_file(e, dialog), auto_upload=True) \
                        .props('color=yellow flat square bordered') \
                        .classes('w-full max-w-xs font-mono')
                    
                    ui.button('CANCEL', on_click=dialog.close).props('flat color=red dense square').classes('mt-4')

        dialog.open()

    async def process_uploaded_file(self, e, dialog):
        e.sender.reset()
        try:
            content_text = ""
            if hasattr(e, 'file') and hasattr(e.file, 'read'):
                possible_coro = e.file.read()
                if asyncio.iscoroutine(possible_coro): content_bytes = await possible_coro
                else: content_bytes = possible_coro
                content_text = content_bytes.decode('utf-8')
            elif hasattr(e, 'content'):
                possible_coro = e.content.read()
                if asyncio.iscoroutine(possible_coro): content_bytes = await possible_coro
                else: content_bytes = possible_coro
                content_text = content_bytes.decode('utf-8')

            if not content_text: ui.notify('READ ERROR', type='negative', classes='rounded-none'); return

            data = yaml.safe_load(content_text)
            target_params = {}
            node_key = self.current_node_name
            if not node_key.startswith('/'): node_key = f'/{node_key}'
            
            if node_key in data and 'ros__parameters' in data[node_key]: target_params = data[node_key]['ros__parameters']
            else:
                for key in data:
                    if isinstance(data[key], dict) and 'ros__parameters' in data[key]: target_params = data[key]['ros__parameters']; break
            
            if not target_params: ui.notify('INVALID YAML STRUCTURE', type='negative', classes='rounded-none'); return

            self.upload_state_container.clear(); self.upload_state_container.delete()
            
            with self.dialog_card:
                with ui.row().classes('w-full items-center justify-between p-2 shrink-0 border-b border-yellow-900/50 bg-yellow-950/10'):
                    ui.label('PREVIEW CONFIG').classes('font-bold text-sm text-yellow-500 tracking-widest font-mono')
                    ui.label(f'{len(target_params)} PARAMS').classes('text-[10px] text-yellow-700 font-mono')

                columns = [
                    {'name': 'param', 'label': 'PARAMETER', 'field': 'param', 'align': 'left', 'classes': 'font-mono text-[10px] text-slate-300 break-all'},
                    {'name': 'value', 'label': 'NEW VALUE', 'field': 'value', 'align': 'left', 'classes': 'font-mono text-[10px] text-green-400 font-bold break-all'},
                ]
                rows = [{'param': k, 'value': str(v)} for k, v in target_params.items()]
                
                with ui.table(columns=columns, rows=rows, row_key='param').classes('w-full flex-grow bg-black border-none').props('dense flat square virtual-scroll'): pass

                with ui.row().classes('w-full justify-end items-center p-2 gap-2 shrink-0 border-t border-slate-800 bg-black'):
                    ui.button('CANCEL', on_click=dialog.close).props('flat color=slate dense square')
                    def confirm_apply():
                        count = 0
                        for name, value in target_params.items():
                            self.rov_node.set_remote_parameter(self.current_node_name.strip('/'), name, value)
                            self.local_params[name] = value 
                            count += 1
                        self.render_filtered_params()
                        dialog.close()
                        ui.notify(f'APPLIED {count} PARAMS', type='positive', color='green', classes='rounded-none')
                    ui.button('CONFIRM & APPLY', on_click=confirm_apply).props('outline color=yellow dense square')

        except Exception as err: ui.notify(f'PARSE ERROR: {err}', type='negative', classes='rounded-none'); print(f"YAML Err: {err}")

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
        self.current_node_name = selected_node
        short_name = selected_node.split('/')[-1]
        self.tab_header._props['label'] = short_name[:12].upper()
        self.tab_header.update()
        self.param_area.clear()
        ui.notify(f'FETCHING: {short_name}', classes='rounded-none bg-slate-800 text-white')
        params = await self.rov_node.get_node_parameters(selected_node.strip('/'))
        if params is None: ui.notify('NODE UNREACHABLE', color='red', classes='rounded-none'); self.local_params={}; return
        self.local_params = params
        self.search_input.value = '' 
        self.render_filtered_params()

    def render_filtered_params(self):
        self.param_area.clear()
        filter_text = self.search_input.value.lower() if self.search_input.value else ""
        if not self.local_params: return
        with self.param_area:
            ui.element('div').classes('h-1') 
            for name in sorted(self.local_params.keys()):
                if filter_text in name.lower():
                    value = self.local_params[name]
                    self._create_param_widget(self.current_node_name, name, value)

    def _create_param_widget(self, node_name, param_name, value):
        with ui.card().classes('w-full bg-slate-900/40 p-1 mb-1 border-l-2 border-slate-700 rounded-none'):
            ui.label(param_name).classes('text-[9px] text-cyan-700 font-mono break-all uppercase mb-0.5 ml-1')
            def update_value(e, n=node_name, p=param_name):
                self.rov_node.set_remote_parameter(n.strip('/'), p, e.value)
                self.local_params[p] = e.value
            
            if isinstance(value, bool): 
                ui.switch(value=value, on_change=update_value).props('dense color=cyan size=xs keep-color').classes('ml-1')
            elif isinstance(value, (int, float)): 
                is_int = isinstance(value, int)
                step = 1 if is_int else 0.000001
                fmt = '%.0f' if is_int else '%.6f'
                ui.number(value=value, format=fmt, step=step, on_change=update_value) \
                    .props('dense outlined square input-class="text-xs font-mono text-white"').classes('w-full bg-black')
            elif isinstance(value, str): 
                ui.input(value=value, on_change=update_value) \
                    .props('dense outlined square input-class="text-xs font-mono text-green-300"').classes('w-full bg-black')


# ==========================================
# MAIN UI BUILDER
# ==========================================
def build_interface(rov_node):
    @ui.page('/')
    def index():
        # Force Dark Mode via Props on the Page/Main div? NiceGUI handles colors globally usually.
        # But we can just use tailwind classes.
        
        # --- HEADER ---
        with ui.header().classes('bg-black border-b border-slate-800 items-center gap-4 h-16 px-4 shrink-0 font-mono'):
            ui.icon('hub', size='sm').classes('text-cyan-500')
            ui.label('BLUEROV2 // COCKPIT').classes('text-md font-bold tracking-[0.2em] text-slate-200')
            
            def set_mode(e):
                rov_node.set_remote_parameter('bluerov2_teleop', 'mode', e.value)
                ui.notify(f'MODE SET: {e.value.upper()}', classes='rounded-none bg-cyan-900 text-white')
            
            ui.select(['manual', 'servoing', 'correction', 'searching'], label='OP_MODE', value='manual', on_change=set_mode) \
                .classes('w-40 text-[10px] rounded-none').props('dense outlined square options-dense bg-color="black" label-color="cyan"')
            
            ui.space()
            
            # ARMING
            def open_arming_dialog():
                current_state = rov_node.armed_state
                action_text = "DISARM" if current_state else "ARM"
                border_col = "red-600"
                with ui.dialog().classes('backdrop-blur-md bg-black/90') as d:
                    d.open()
                    def handle_key(e: events.KeyEventArguments):
                        if e.key == ' ' and e.action.keydown:
                            new_state = not current_state
                            success, msg = rov_node.arm_vehicle(new_state)
                            if success:
                                ui.notify(f'VEHICLE {"ARMED" if new_state else "DISARMED"}', type='positive', color='red' if new_state else 'green', classes='rounded-none')
                                arm_btn.props(f'color={"red" if new_state else "green"} label={"DISARM" if new_state else "ARM"}')
                            else: ui.notify(f'FAIL: {msg}', type='negative', classes='rounded-none')
                            d.close()
                    ui.keyboard(on_key=handle_key)
                    with ui.card().classes(f'bg-black border-2 border-{border_col} items-center p-8 w-[500px] shadow-[0_0_50px_rgba(255,0,0,0.2)] rounded-none'):
                        ui.icon('gpp_maybe', size='3em').classes(f'text-{border_col} mb-4 animate-pulse')
                        ui.label(f':: SYSTEM {action_text} SEQUENCE ::').classes(f'text-2xl font-bold text-{border_col} mb-4 tracking-widest font-mono')
                        ui.label('[ PRESS SPACEBAR TO CONFIRM ]').classes('text-sm text-white font-mono bg-slate-900 px-4 py-2 border border-slate-700')
                        ui.button('ABORT', on_click=d.close).props('flat color=slate square').classes('mt-8 w-full')

            arm_btn = ui.button('ARM', on_click=open_arming_dialog) \
                .props('color=green outline square icon=lock_open') \
                .classes('font-bold tracking-widest px-6 border-2 font-mono rounded-none')

            ui.space()

            # TELEMETRY
            with ui.row().classes('items-center gap-0 border border-slate-800 bg-slate-900/50 px-2 py-1 h-8'):
                ui.icon('battery_charging_full', size='xs').classes('text-orange-400 mr-2')
                batt_label = ui.label('00.0 V').classes('text-orange-400 text-sm font-mono font-bold mr-3')
                ui.element('div').classes('w-[1px] h-4 bg-slate-700 mx-1') 
                ui.icon('vertical_align_bottom', size='xs').classes('text-cyan-400 mr-2 ml-3')
                depth_label = ui.label('00.00 m').classes('text-cyan-400 text-sm font-mono font-bold')

            def update_header_stats():
                batt_label.text = rov_node.battery_voltage
                depth_label.text = rov_node.depth_reading
            ui.timer(1.0, update_header_stats)

        # --- MAIN LAYOUT (MIN DIMENSIONS ADDED) ---
        # Added min-w and min-h to force scrolling on small screens
        with ui.row().classes('w-full h-[calc(100vh-64px)] no-wrap gap-0 overflow-hidden min-w-[1200px] min-h-[800px] font-mono'):
            
            # --- SPLITTER 1 ---
            with ui.splitter(horizontal=False, value=25, limits=[15, 40]).classes('w-full h-full bg-black') as main_split:
                main_split.props('separator-class="bg-slate-800" separator-style="width: 1px"')
                
                # LEFT
                with main_split.before:
                    with ui.column().classes('w-full h-full bg-black border-r border-slate-800 p-0 overflow-hidden'):
                        with ui.splitter(horizontal=True, value=50, limits=[20, 80]).classes('w-full h-full') as left_sub_split:
                            left_sub_split.props('separator-class="bg-slate-800" separator-style="height: 1px"')
                            with left_sub_split.before:
                                with ui.column().classes('w-full h-full flex flex-col overflow-hidden'):
                                    with ui.row().classes('w-full h-8 bg-slate-950 items-center pl-1 pr-1 gap-0 border-b border-slate-800 overflow-x-auto'):
                                        with ui.row().classes('flex-grow h-full overflow-x-auto no-wrap items-end pr-2'):
                                            with ui.tabs().classes('h-full text-[10px] text-slate-500 active-text-cyan-400') as echo_tabs_top: pass
                                        ui.button(icon='add', on_click=lambda: EchoTab(rov_node, echo_tabs_top, echo_panels_top)).props('flat square dense size=sm').classes('text-slate-500 hover:text-cyan-400')
                                    with ui.tab_panels(echo_tabs_top, value=None).classes('w-full flex-grow bg-black overflow-hidden rounded-none') as echo_panels_top: pass
                                    EchoTab(rov_node, echo_tabs_top, echo_panels_top)
                            with left_sub_split.after:
                                with ui.column().classes('w-full h-full flex flex-col overflow-hidden'):
                                    with ui.row().classes('w-full h-8 bg-slate-950 items-center pl-1 pr-1 gap-0 border-b border-slate-800 overflow-x-auto'):
                                        with ui.row().classes('flex-grow h-full overflow-x-auto no-wrap items-end pr-2'):
                                            with ui.tabs().classes('h-full text-[10px] text-slate-500 active-text-cyan-400') as echo_tabs_bot: pass
                                        ui.button(icon='add', on_click=lambda: EchoTab(rov_node, echo_tabs_bot, echo_panels_bot)).props('flat square dense size=sm').classes('text-slate-500 hover:text-cyan-400')
                                    with ui.tab_panels(echo_tabs_bot, value=None).classes('w-full flex-grow bg-black overflow-hidden rounded-none') as echo_panels_bot: pass
                                    EchoTab(rov_node, echo_tabs_bot, echo_panels_bot)

                # --- SPLITTER 2 ---
                with main_split.after:
                    with ui.splitter(horizontal=False, value=66.66, limits=[40, 85]).classes('w-full h-full') as right_split:
                        right_split.props('separator-class="bg-slate-800" separator-style="width: 1px"')
                        
                        # CENTER
                        with right_split.before:
                            with ui.column().classes('w-full h-full bg-[#050505] p-0 overflow-hidden relative'):
                                with ui.row().classes('absolute top-2 left-1/2 transform -translate-x-1/2 z-50 bg-black/80 border border-slate-700 px-2 py-1 gap-2'):
                                    mode_switch = ui.toggle({1: 'SINGLE', 2: 'DUAL', 3: 'CONFIG'}, value=1) \
                                        .props('dense no-caps unelevated toggle-color="cyan-900" text-color="slate-400"') \
                                        .classes('border border-slate-700 rounded-none')

                                sources = {'RAW': 'latest_image_raw', 'DETECT': 'latest_image_processed', 'SERVO': 'latest_image_servo'}

                                with ui.splitter(horizontal=True, value=100).classes('w-full h-full relative') as cam_split:
                                    cam_split.props('separator-class="bg-cyan-900" separator-style="height: 2px"')
                                    with cam_split.before:
                                        with ui.column().classes('w-full h-full p-0 relative bg-black') as slot_1:
                                            ui.element('div').classes('absolute top-0 left-0 w-full h-full border-2 border-slate-800 pointer-events-none z-0 opacity-50')
                                            ui.element('div').classes('absolute top-1/2 left-1/2 w-4 h-4 border-l border-t border-cyan-500/50 -translate-x-1/2 -translate-y-1/2 z-0')
                                            ui.element('div').classes('absolute top-1/2 left-1/2 w-4 h-4 border-r border-b border-cyan-500/50 -translate-x-1/2 -translate-y-1/2 z-0')
                                            with ui.row().classes('absolute top-2 left-2 z-20 bg-black/80 border-l-2 border-cyan-500 px-2'):
                                                ui.label('CAM_01').classes('text-[10px] font-bold text-cyan-500 mr-2')
                                                sel_1 = ui.select(list(sources.keys()), value='RAW').props('dense borderless options-dense behavior="menu"').classes('w-24 text-[10px]')
                                            img_1 = ui.interactive_image().classes('w-full h-full object-contain z-10')
                                    with cam_split.after:
                                        with ui.column().classes('w-full h-full p-0 relative bg-black') as slot_2:
                                            ui.element('div').classes('absolute top-0 left-0 w-full h-full border-2 border-slate-800 pointer-events-none z-0 opacity-50')
                                            with ui.row().classes('absolute top-2 left-2 z-20 bg-black/80 border-l-2 border-yellow-500 px-2'):
                                                ui.label('CAM_02').classes('text-[10px] font-bold text-yellow-500 mr-2')
                                                sel_2 = ui.select(list(sources.keys()), value='DETECT').props('dense borderless options-dense behavior="menu"').classes('w-24 text-[10px]')
                                            img_2 = ui.interactive_image().classes('w-full h-full object-contain z-10')

                                with ui.column().classes('w-full h-full bg-black border-2 border-dashed border-yellow-900/50 hidden relative') as center_config_panel:
                                    ui.label('AUXILIARY CONFIGURATION').classes('absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-slate-800 text-6xl font-bold tracking-widest pointer-events-none select-none')
                                    with ui.row().classes('w-full h-10 bg-slate-950 items-center pl-2 gap-2 border-b border-yellow-900/30 z-10'):
                                        ui.icon('settings_applications', size='sm').classes('text-yellow-600')
                                        with ui.row().classes('flex-grow h-full overflow-x-auto no-wrap items-end'):
                                            with ui.tabs().classes('h-full text-[10px] text-slate-500 active-text-yellow-500') as c_tabs: pass 
                                        ui.button(icon='add', on_click=lambda: NodeInspectorTab(rov_node, c_tabs, c_panels)).props('flat square dense size=sm').classes('text-yellow-600')
                                    with ui.tab_panels(c_tabs, value=None).classes('w-full flex-grow bg-transparent overflow-hidden z-10 rounded-none') as c_panels: pass
                                    NodeInspectorTab(rov_node, c_tabs, c_panels)

                                def update_layout():
                                    val = mode_switch.value
                                    if val == 1:
                                        cam_split.visible = True; center_config_panel.visible = False
                                        cam_split.set_value(100)
                                    elif val == 2:
                                        cam_split.visible = True; center_config_panel.visible = False
                                        cam_split.set_value(50)
                                    elif val == 3:
                                        cam_split.visible = False; center_config_panel.visible = True

                                mode_switch.on_value_change(update_layout)
                                ui.timer(0.1, update_layout, once=True)

                                def update_frame():
                                    if mode_switch.value != 3:
                                        attr1 = sources.get(sel_1.value); data1 = getattr(rov_node, attr1, None)
                                        if data1: img_1.set_source(data1)
                                        if mode_switch.value == 2:
                                            attr2 = sources.get(sel_2.value); data2 = getattr(rov_node, attr2, None)
                                            if data2: img_2.set_source(data2)
                                ui.timer(0.033, update_frame)

                        # RIGHT
                        with right_split.after:
                            with ui.column().classes('w-full h-full bg-black border-l border-slate-800 flex flex-col overflow-hidden'):
                                with ui.row().classes('w-full h-8 bg-slate-950 items-center pl-1 pr-1 gap-0 border-b border-slate-800 overflow-x-auto'):
                                    with ui.row().classes('flex-grow h-full overflow-x-auto no-wrap items-end pr-2'):
                                        with ui.tabs().classes('h-full text-[10px] text-slate-500 active-text-green-400') as r_tabs: pass 
                                    ui.button(icon='add', on_click=lambda: NodeInspectorTab(rov_node, r_tabs, r_panels)).props('flat square dense size=sm').classes('text-green-600 hover:text-green-400')
                                with ui.tab_panels(r_tabs, value=None).classes('w-full flex-grow bg-black overflow-hidden rounded-none') as r_panels: pass
                                NodeInspectorTab(rov_node, r_tabs, r_panels)