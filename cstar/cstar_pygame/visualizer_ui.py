import pygame

class UIState:
    def __init__(self, window_size, panel_width):
        self.window_size = window_size
        self.panel_width = panel_width
        self.buttons = {}
        self.slider_rect = pygame.Rect(window_size + 20, 200, 200, 20)
        self.slider_handle = pygame.Rect(window_size + 20, 195, 10, 30)
        self.slider_value = 10
        self.dragging_slider = False

def setup_ui_elements(ui, colors):
    button_width = 120
    button_height = 30
    start_x = ui.window_size + 20
    start_y = 220
    button_configs = [
        ('start', 'Start C*', start_y),
        ('pause', 'Pause', start_y + 40),
        ('reset', 'Reset', start_y + 80),
    ]
    for btn_id, text, y_pos in button_configs:
        ui.buttons[btn_id] = {
            'rect': pygame.Rect(start_x, y_pos, button_width, button_height),
            'text': text,
            'enabled': True if btn_id != 'pause' else False,
            'color': colors['button_normal'],
        }

def handle_mouse_hover(ui, colors, pos):
    for btn_data in ui.buttons.values():
        if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
            btn_data['color'] = colors['button_hover']
        else:
            btn_data['color'] = colors['button_normal']

def update_slider(ui, mouse_x):
    relative_x = mouse_x - ui.slider_rect.x
    relative_x = max(0, min(relative_x, ui.slider_rect.width))
    ui.slider_handle.x = ui.slider_rect.x + relative_x - 5
    ui.slider_value = int((relative_x / ui.slider_rect.width) * 200)
    step_size = ui.slider_value / 1000.0
    return step_size
