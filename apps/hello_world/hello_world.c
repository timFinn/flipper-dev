#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>

typedef struct {
    FuriMessageQueue* input_queue;
} HelloWorldApp;

static void hello_world_draw_callback(Canvas* canvas, void* ctx) {
    UNUSED(ctx);
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 20, AlignCenter, AlignCenter, "Hello, Flipper!");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 40, AlignCenter, AlignCenter, "Press BACK to exit");
}

static void hello_world_input_callback(InputEvent* event, void* ctx) {
    furi_assert(ctx);
    HelloWorldApp* app = ctx;
    furi_message_queue_put(app->input_queue, event, FuriWaitForever);
}

int32_t hello_world_app(void* p) {
    UNUSED(p);

    HelloWorldApp app = {
        .input_queue = furi_message_queue_alloc(8, sizeof(InputEvent)),
    };

    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, hello_world_draw_callback, &app);
    view_port_input_callback_set(view_port, hello_world_input_callback, &app);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    InputEvent event;
    while(furi_message_queue_get(app.input_queue, &event, FuriWaitForever) == FuriStatusOk) {
        if(event.type == InputTypeShort && event.key == InputKeyBack) {
            break;
        }
    }

    view_port_enabled_set(view_port, false);
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    furi_message_queue_free(app.input_queue);

    return 0;
}
