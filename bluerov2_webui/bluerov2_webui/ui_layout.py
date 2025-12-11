from nicegui import ui, events
import uuid
import asyncio
import json
import yaml
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
# COMPONENT: ECHO TAB (Left Panel)
# ==========================================
class EchoTab:
    def __init__(self, rov_node, tabs_container, panels_container):
        self.rov_node = rov_node
        self.id = uuid.uuid4().hex
        self.active_keys = set() 
        self.latest_full_msg = None 
        
        with tabs_container:
            self.tab_header = ui.tab(name=self.id, label='Echo') \
                .classes('h-full text-xs min-w-[60px] mx-1 rounded-t border-t border-l border-r border-transparent hover:border-slate-700')
        
        with panels_container:
            self.panel = ui.tab_panel(name=self.id).classes('w-full h-full p-0 flex flex-col bg-slate-900 overflow-hidden min-h-0 min-w-0')
            with self.panel:
                self._build_ui()
                
        tabs_container.set_value(self.id)
        ui.timer(0.0, self._refresh_topics, once=True)

    def _build_ui(self):
        with ui.row().classes('w-full items-center gap-1 p-2 bg-slate-800 border-b border-slate-700 shrink-0'):
            self.topic_select = ui.select([], label='Topic', with_input=True) \
                .classes('flex-grow text-xs min-w-[80px]').props('dense options-dense')
            self.topic_select.on_value_change(self._on_topic_change)
            
            self.view_mode = ui.switch('Table').props('dense color=green size=xs').classes('text-xs text-slate-400 mr-2')
            
            ui.button(icon='refresh', on_click=self._refresh_topics).props('flat dense size=sm').classes('text-slate-400')
            ui.button(icon='settings', on_click=self._open_filter_dialog).props('flat dense size=sm').classes('text-slate-400')
            ui.button(icon='close', on_click=self.close).props('flat dense size=sm').classes('text-red-400')

        self.text_scroll = ui.scroll_area().classes('w-full flex-grow bg-black/30 p-2 min-h-0')
        with self.text_scroll:
            self.content_label = ui.label('(Select a topic to echo)') \
                .classes('text-xs font-mono text-slate-500 whitespace-pre-wrap break-all w-full')
        
        columns = [
            {'name': 'key', 'label': 'Field', 'field': 'key', 'align': 'left', 'classes': 'break-all whitespace-pre-wrap text-[10px] font-mono text-slate-400', 'style': 'width: 40%'},
            {'name': 'val', 'label': 'Value', 'field': 'val', 'align': 'left', 'classes': 'break-all whitespace-pre-wrap text-[10px] font-mono text-green-400', 'style': 'width: 60%'},
        ]
        
        with ui.column().classes('w-full flex-grow min-h-0 p-0 relative max-w-full overflow-hidden') as self.table_container:
            self.content_table = ui.table(columns=columns, rows=[], row_key='key') \
                .classes('w-full h-full bg-slate-800 border-none') \
                .props('dense flat square hide-bottom wrap-cells') 
            
        self.table_container.bind_visibility_from(self.view_mode, 'value')
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
                    def clear_filter(): self.active_keys = set(); dialog.close(); ui.notify('Filter cleared')
                    def apply_filter(): self.active_keys = {k for k, cb in self.checkboxes.items() if cb.value}; dialog.close(); ui.notify(f'Filter applied')
                    ui.button('Reset', on_click=clear_filter).props('flat color=red dense')
                    ui.button('Apply Filter', on_click=apply_filter).props('color=green dense unelevated').classes('px-6')
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
            self.content_label.text = "(Select a topic to echo)"
            self.content_label.classes(replace='text-slate-500')
            self.content_table.rows = []
            return
        self.active_keys = set()
        self.latest_full_msg = None
        parts = e.value.split('|')
        topic = parts[0].strip(); type_str = parts[1].strip()
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
        self.current_node_name = None
        self.local_params = {} 
        
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
            ui.button(icon='file_upload', on_click=self.open_load_dialog).props('flat dense size=sm').classes('text-yellow-400').tooltip('Load Params')
            ui.button(icon='save', on_click=self.prompt_save_filename).props('flat dense size=sm').classes('text-blue-400').tooltip('Save Params')
            ui.button(icon='refresh', on_click=self.refresh_list).props('flat dense size=sm').classes('text-slate-400')
            ui.button(icon='close', on_click=self.close).props('flat dense size=sm').classes('text-red-400 hover:bg-slate-700')

        with ui.row().classes('w-full p-2 bg-slate-800/50 border-b border-slate-700'):
            self.search_input = ui.input(placeholder='Search parameters...').props('dense outlined rounded item-aligned input-class="text-xs text-white"').classes('w-full text-xs').on_value_change(self.render_filtered_params)

        self.param_area = ui.scroll_area().classes('w-full flex-grow p-2 bg-slate-900/30')

    # --- SAVE ---
    def prompt_save_filename(self):
        if not self.current_node_name or not self.local_params:
            ui.notify('No parameters loaded to save', type='warning')
            return
        clean_name = self.current_node_name.strip('/').replace('/', '_')
        default_name = f"params_{clean_name}.yaml"
        with ui.dialog().classes('backdrop-blur-sm bg-black/60') as dialog:
            with ui.card().classes('bg-slate-800 text-white w-96'):
                ui.label('Save Parameters').classes('font-bold text-lg')
                filename_input = ui.input('Filename', value=default_name).classes('w-full')
                def perform_save():
                    final_name = filename_input.value
                    if not final_name.endswith('.yaml'): final_name += '.yaml'
                    self.save_parameters_to_file(final_name); dialog.close()
                with ui.row().classes('w-full justify-end mt-4'):
                    ui.button('Cancel', on_click=dialog.close).props('flat color=red dense')
                    ui.button('Download', on_click=perform_save).props('color=blue dense unelevated')
        dialog.open()

    def save_parameters_to_file(self, filename):
        node_key = self.current_node_name
        if not node_key.startswith('/'): node_key = f'/{node_key}'
        yaml_structure = {node_key: {'ros__parameters': self.local_params}}
        try:
            yaml_content = yaml.dump(yaml_structure, default_flow_style=False)
            ui.download(yaml_content.encode('utf-8'), filename)
            ui.notify(f'Downloaded {filename}', type='positive')
        except Exception as e: ui.notify(f'Error: {e}', type='negative')

    # --- LOAD (PREVIEW) ---
    def open_load_dialog(self):
        if not self.current_node_name:
            ui.notify('Please select a node first', type='warning')
            return

        with ui.dialog().classes('backdrop-blur-sm bg-black/60') as dialog:
            self.dialog_card = ui.card().classes('bg-slate-800 text-white w-[600px] h-[500px] flex flex-col p-0')
            
            with self.dialog_card:
                with ui.column().classes('w-full h-full justify-center items-center p-8') as self.upload_state_container:
                    ui.icon('upload_file', size='xl').classes('text-slate-500 mb-4')
                    ui.label('Upload YAML Configuration').classes('text-xl font-bold mb-2')
                    ui.label(f'Target Node: {self.current_node_name}').classes('text-sm text-slate-400 mb-6 font-mono')
                    
                    ui.upload(on_upload=lambda e: self.process_uploaded_file(e, dialog), auto_upload=True) \
                        .props('color=blue flat') \
                        .classes('w-full max-w-xs')
                    
                    ui.button('Cancel', on_click=dialog.close).props('flat color=red dense').classes('mt-4')

        dialog.open()

    async def process_uploaded_file(self, e, dialog):
        e.sender.reset()
        
        try:
            content_text = ""
            # ASYNC FILE READER
            if hasattr(e, 'file'):
                if hasattr(e.file, 'read'):
                    possible_coro = e.file.read()
                    if asyncio.iscoroutine(possible_coro): content_bytes = await possible_coro
                    else: content_bytes = possible_coro
                    content_text = content_bytes.decode('utf-8')
            elif hasattr(e, 'content'):
                possible_coro = e.content.read()
                if asyncio.iscoroutine(possible_coro): content_bytes = await possible_coro
                else: content_bytes = possible_coro
                content_text = content_bytes.decode('utf-8')

            if not content_text: ui.notify('Error: Could not read file content.', type='negative'); return

            data = yaml.safe_load(content_text)
            target_params = {}
            node_key = self.current_node_name
            if not node_key.startswith('/'): node_key = f'/{node_key}'
            
            if node_key in data and 'ros__parameters' in data[node_key]:
                target_params = data[node_key]['ros__parameters']
            else:
                for key in data:
                    if isinstance(data[key], dict) and 'ros__parameters' in data[key]:
                        target_params = data[key]['ros__parameters']
                        break
            
            if not target_params: ui.notify('Invalid YAML: Could not find ros__parameters', type='negative'); return

            self.upload_state_container.clear()
            self.upload_state_container.delete()
            
            with self.dialog_card:
                with ui.row().classes('w-full items-center justify-between p-4 shrink-0 border-b border-slate-700'):
                    ui.label('Preview Configuration').classes('font-bold text-lg')
                    ui.label(f'{len(target_params)} Params found').classes('text-xs text-green-400')

                columns = [
                    {'name': 'param', 'label': 'Parameter', 'field': 'param', 'align': 'left', 'classes': 'font-mono text-xs text-slate-300 break-all'},
                    {'name': 'value', 'label': 'New Value', 'field': 'value', 'align': 'left', 'classes': 'font-mono text-xs text-green-300 font-bold break-all'},
                ]
                rows = [{'param': k, 'value': str(v)} for k, v in target_params.items()]
                
                with ui.table(columns=columns, rows=rows, row_key='param').classes('w-full flex-grow bg-slate-900 border-none').props('dense flat virtual-scroll'):
                    pass

                with ui.row().classes('w-full justify-end items-center p-4 gap-4 shrink-0 border-t border-slate-700 bg-slate-800'):
                    ui.button('Cancel', on_click=dialog.close).props('flat color=red dense')
                    def confirm_apply():
                        count = 0
                        for name, value in target_params.items():
                            self.rov_node.set_remote_parameter(self.current_node_name.strip('/'), name, value)
                            self.local_params[name] = value 
                            count += 1
                        self.render_filtered_params()
                        dialog.close()
                        ui.notify(f'Successfully applied {count} parameters', type='positive')
                    ui.button('CONFIRM & APPLY', on_click=confirm_apply).props('color=green dense unelevated')

        except Exception as err: ui.notify(f'Parse Error: {err}', type='negative'); print(f"Error parsing YAML: {err}")

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
        self.tab_header._props['label'] = short_name
        self.tab_header.update()
        self.param_area.clear()
        ui.notify(f'Loading {short_name}...')
        params = await self.rov_node.get_node_parameters(selected_node.strip('/'))
        if params is None: ui.notify('Node not reachable', color='red'); self.local_params={}; return
        self.local_params = params
        self.search_input.value = '' 
        self.render_filtered_params()

    def render_filtered_params(self):
        self.param_area.clear()
        filter_text = self.search_input.value.lower() if self.search_input.value else ""
        if not self.local_params: return
        with self.param_area:
            ui.element('div').classes('h-2') 
            for name in sorted(self.local_params.keys()):
                if filter_text in name.lower():
                    value = self.local_params[name]
                    self._create_param_widget(self.current_node_name, name, value)

    def _create_param_widget(self, node_name, param_name, value):
        with ui.card().classes('w-full bg-slate-700 p-2 mb-2 border-l-4 border-slate-500'):
            ui.label(param_name).classes('text-[10px] text-slate-400 font-mono break-all uppercase mb-1')
            def update_value(e, n=node_name, p=param_name):
                self.rov_node.set_remote_parameter(n.strip('/'), p, e.value)
                self.local_params[p] = e.value
            if isinstance(value, bool): ui.switch(value=value, on_change=update_value).props('dense')
            elif isinstance(value, (int, float)): 
                is_int = isinstance(value, int)
                step = 1 if is_int else 0.000001
                fmt = '%.0f' if is_int else '%.6f'
                ui.number(value=value, format=fmt, step=step, on_change=update_value).classes('w-full bg-slate-800 rounded px-2 text-white text-sm')
            elif isinstance(value, str): ui.input(value=value, on_change=update_value).classes('w-full text-white text-sm')


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
            ui.select(['manual', 'servoing', 'correction', 'searching'], label='Teleop Mode', value='manual', on_change=set_mode).classes('w-32 bg-slate-700 rounded-md text-white')
            ui.space()
            ui.label('System: ONLINE').classes('text-green-400 text-xs font-mono')

        # --- LAYOUT (ADJUSTABLE WIDTHS - HORIZONTAL FALSE) ---
        with ui.row().classes('w-full h-[calc(100vh-65px)] no-wrap gap-0 overflow-x-auto'):
            
            # --- SPLITTER 1 (Vertical Divide): LEFT vs REST ---
            with ui.splitter(horizontal=False, value=25, limits=[15, 40]).classes('w-full h-full') as main_split:
                
                # --- LEFT PANEL ---
                with main_split.before:
                    with ui.column().classes('w-full h-full bg-slate-800 border-r border-slate-700 p-0 overflow-hidden min-w-0'):
                        # Left Panel Inner Split (Top/Bottom)
                        with ui.splitter(horizontal=True, value=50, limits=[20, 80]).classes('w-full h-full') as left_sub_split:
                            with left_sub_split.before:
                                with ui.column().classes('w-full h-full flex flex-col overflow-hidden min-h-0 min-w-0'):
                                    with ui.row().classes('w-full h-8 bg-slate-900 items-center pl-2 pr-2 gap-2 no-wrap overflow-y-hidden shrink-0'):
                                        with ui.row().classes('flex-grow h-full overflow-x-auto overflow-y-hidden no-wrap items-center pr-8'):
                                            with ui.tabs().classes('h-full text-xs text-slate-400') as echo_tabs_top: pass
                                        ui.button(icon='add', on_click=lambda: EchoTab(rov_node, echo_tabs_top, echo_panels_top)).props('flat round dense').classes('text-green-400 shrink-0')
                                    with ui.tab_panels(echo_tabs_top, value=None).classes('w-full flex-grow bg-slate-800 overflow-hidden min-h-0 min-w-0') as echo_panels_top: pass
                                    EchoTab(rov_node, echo_tabs_top, echo_panels_top)
                            with left_sub_split.after:
                                with ui.column().classes('w-full h-full flex flex-col border-t border-slate-700 overflow-hidden min-h-0 min-w-0'):
                                    with ui.row().classes('w-full h-8 bg-slate-900 items-center pl-2 pr-2 gap-2 no-wrap overflow-y-hidden shrink-0'):
                                        with ui.row().classes('flex-grow h-full overflow-x-auto overflow-y-hidden no-wrap items-center pr-8'):
                                            with ui.tabs().classes('h-full text-xs text-slate-400') as echo_tabs_bot: pass
                                        ui.button(icon='add', on_click=lambda: EchoTab(rov_node, echo_tabs_bot, echo_panels_bot)).props('flat round dense').classes('text-green-400 shrink-0')
                                    with ui.tab_panels(echo_tabs_bot, value=None).classes('w-full flex-grow bg-slate-800 overflow-hidden min-h-0 min-w-0') as echo_panels_bot: pass
                                    EchoTab(rov_node, echo_tabs_bot, echo_panels_bot)

                # --- SPLITTER 2 (Vertical Divide): CENTER vs RIGHT ---
                with main_split.after:
                    with ui.splitter(horizontal=False, value=66.66, limits=[40, 85]).classes('w-full h-full') as right_split:
                        
                        # --- CENTER PANEL (VIDEO) ---
                        with right_split.before:
                            with ui.column().classes('w-full h-full p-4 items-center bg-black min-w-0 overflow-hidden'):
                                # 1. View Mode Toggle
                                mode_switch = ui.toggle({1: 'Single View', 2: 'Dual View'}, value=1) \
                                    .classes('bg-slate-700 text-white shadow-lg shrink-0 mb-2')

                                # 2. Source Options
                                sources = {
                                    'Raw Feed': 'latest_image_raw',
                                    'Detections': 'latest_image_processed',
                                    'Visual Servo': 'latest_image_servo'
                                }

                                # 3. Video Splitter (Horizontal=True -> Top/Bottom)
                                with ui.splitter(horizontal=True, value=100).classes('w-full h-full relative border-2 border-slate-800') as cam_split:
                                    
                                    # TOP SLOT
                                    with cam_split.before:
                                        with ui.column().classes('w-full h-full p-0 gap-0 relative overflow-hidden bg-black/50') as slot_1:
                                            with ui.row().classes('absolute top-0 left-0 w-full z-10 p-1 bg-black/60 items-center justify-between'):
                                                ui.label('CAM 1').classes('text-xs font-bold text-slate-400 ml-2')
                                                sel_1 = ui.select(list(sources.keys()), value='Raw Feed').props('dense options-dense filled').classes('w-32 text-xs bg-slate-800')
                                            img_1 = ui.interactive_image().classes('w-full h-full object-contain')

                                    # BOTTOM SLOT
                                    with cam_split.after:
                                        with ui.column().classes('w-full h-full p-0 gap-0 relative overflow-hidden bg-black/50 border-t-2 border-blue-600') as slot_2:
                                            with ui.row().classes('absolute top-0 left-0 w-full z-10 p-1 bg-black/60 items-center justify-between'):
                                                ui.label('CAM 2').classes('text-xs font-bold text-blue-400 ml-2')
                                                sel_2 = ui.select(list(sources.keys()), value='Detections').props('dense options-dense filled').classes('w-32 text-xs bg-slate-800')
                                            img_2 = ui.interactive_image().classes('w-full h-full object-contain')

                                # 4. Layout Logic
                                def update_layout():
                                    if mode_switch.value == 1:
                                        # Single Mode: Top takes 100%
                                        cam_split.set_value(100)
                                        # Hide the drag handle via CSS
                                        cam_split.classes(add='pointer-events-none') 
                                        # Note: We don't need to manually hide slot_2 because value=100 pushes it off
                                    else:
                                        # Dual Mode: Split 50/50
                                        cam_split.set_value(50)
                                        cam_split.classes(remove='pointer-events-none')

                                mode_switch.on_value_change(update_layout)
                                ui.timer(0.1, update_layout, once=True)

                                # 5. Data Loop
                                def update_frame():
                                    # Update Cam 1
                                    attr1 = sources.get(sel_1.value)
                                    data1 = getattr(rov_node, attr1, None)
                                    if data1: img_1.set_source(data1)

                                    # Update Cam 2 (only if dual mode)
                                    if mode_switch.value == 2:
                                        attr2 = sources.get(sel_2.value)
                                        data2 = getattr(rov_node, attr2, None)
                                        if data2: img_2.set_source(data2)

                                ui.timer(0.033, update_frame)

                        # --- RIGHT PANEL ---
                        with right_split.after:
                            with ui.column().classes('w-full h-full border-l border-slate-700 bg-slate-800 flex flex-col min-w-0'):
                                with ui.row().classes('w-full h-12 bg-slate-900 items-center pl-2 pr-2 gap-2 no-wrap overflow-y-hidden shrink-0'):
                                    with ui.row().classes('flex-grow h-full overflow-x-auto overflow-y-hidden no-wrap items-center pr-8'):
                                        with ui.tabs().classes('h-full text-xs text-slate-400') as r_tabs: pass 
                                    ui.button(icon='add', on_click=lambda: NodeInspectorTab(rov_node, r_tabs, r_panels)).props('flat round dense').classes('text-green-400 shrink-0')
                                with ui.tab_panels(r_tabs, value=None).classes('w-full flex-grow bg-slate-800 overflow-hidden min-h-0') as r_panels: pass
                                NodeInspectorTab(rov_node, r_tabs, r_panels)