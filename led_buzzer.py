import atexit
import os
import sys
import threading
import time
from typing import Optional, Tuple

_server_dir = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
)
if _server_dir not in sys.path:
    sys.path.insert(0, _server_dir)

_lock = threading.Lock()
_fx_thread: Optional[threading.Thread] = None
_fx_stop: Optional[threading.Event] = None
_fx_mode: Optional[str] = None
_fx_color: Optional[Tuple[int, int, int]] = None

_led = None
_buzzer = None
_buzzer_ok = False
_buzzer_pin: Optional[int] = None
_led_ok = False
_BUZZER_PINS = (17,)


def _can_use_ws281x() -> bool:
    try:
        return os.access('/dev/mem', os.R_OK | os.W_OK)
    except Exception:
        return False


def _init_led() -> None:
    global _led, _led_ok
    if _led is not None:
        return
    try:
        from Led import Led
        if not _can_use_ws281x():
            try:
                import parameter
                if hasattr(parameter, 'ParameterManager'):
                    original = parameter.ParameterManager.get_raspberry_pi_version

                    def _patched_get_pi_version(self):
                        try:
                            v = original(self)
                        except Exception:
                            v = 1
                        try:
                            pcb = self.get_pcb_version()
                        except Exception:
                            pcb = 1
                        if pcb == 1 and v == 1:
                            return 2
                        return v
                    parameter.ParameterManager.get_raspberry_pi_version = (
                        _patched_get_pi_version
                    )
            except Exception:
                pass
        _led = Led()
        _led_ok = bool(getattr(_led, 'is_support_led_function', True))
    except Exception:
        _led = None
        _led_ok = False


def _init_buzzer() -> None:
    global _buzzer, _buzzer_ok, _buzzer_pin
    if _buzzer is not None:
        return
    try:
        from gpiozero import Buzzer as _GpioBuzzer
        last_exc: Exception | None = None
        for pin in _BUZZER_PINS:
            try:
                _buzzer = _GpioBuzzer(int(pin))
                _buzzer_ok = True
                _buzzer_pin = int(pin)
                return
            except Exception as e:
                last_exc = e
                _buzzer = None
                _buzzer_ok = False
                _buzzer_pin = None
        if last_exc is not None:
            raise last_exc
    except Exception:
        _buzzer = None
        _buzzer_ok = False
        _buzzer_pin = None


def _close_buzzer() -> None:
    global _buzzer, _buzzer_ok, _buzzer_pin
    if _buzzer is None:
        _buzzer_ok = False
        _buzzer_pin = None
        return
    try:
        try:
            _buzzer.off()
        except Exception:
            pass
        try:
            _buzzer.close()
        except Exception:
            pass
    finally:
        _buzzer = None
        _buzzer_ok = False
        _buzzer_pin = None


def _get_strip():
    _init_led()
    if not _led_ok or _led is None:
        return None
    return getattr(_led, 'strip', None)


def _strip_count(strip) -> int:
    try:
        if strip is None:
            return 0
        if hasattr(strip, 'get_led_count'):
            return int(strip.get_led_count())
        if hasattr(strip, 'numPixels'):
            return int(strip.numPixels())
    except Exception:
        return 0
    return 0


def _set_pixel(strip, i: int, rgb: Tuple[int, int, int]) -> None:
    r, g, b = [int(x) for x in rgb]
    if hasattr(strip, 'setPixelColor'):
        strip.setPixelColor(i, (r, g, b))
    else:
        strip.set_led_rgb_data(i, [r, g, b])


def _show(strip) -> None:
    strip.show()


def _set_color(rgb: Tuple[int, int, int]) -> None:
    _init_led()
    if not _led_ok or _led is None:
        return
    try:
        strip = _get_strip()
        count = _strip_count(strip)
        if strip is None or count <= 0:
            return
        for i in range(count):
            _set_pixel(strip, i, rgb)
        _show(strip)
    except Exception:
        pass


def _stop_fx() -> None:
    global _fx_thread, _fx_stop, _fx_mode, _fx_color
    with _lock:
        stop = _fx_stop
        thread = _fx_thread
        _fx_stop = None
        _fx_thread = None
        _fx_mode = None
        _fx_color = None
        if stop is not None:
            stop.set()
    if thread is not None and thread.is_alive():
        deadline = time.time() + 1.0
        while thread.is_alive() and time.time() < deadline:
            thread.join(timeout=0.1)


def _clear_if_current(stop: threading.Event) -> None:
    global _fx_thread, _fx_stop, _fx_mode, _fx_color
    with _lock:
        if _fx_stop is not stop:
            return
        _fx_stop = None
        _fx_thread = None
        _fx_mode = None
        _fx_color = None


def _wrap_runner(stop: threading.Event, target, args) -> None:
    try:
        target(*args)
    finally:
        _clear_if_current(stop)


def _run_breathe(stop: threading.Event, base: Tuple[int, int, int]) -> None:
    _init_led()
    phase = 0
    while not stop.is_set():
        level = phase if phase <= 100 else 200 - phase
        scale = max(5, level) / 100.0
        r = int(base[0] * scale)
        g = int(base[1] * scale)
        b = int(base[2] * scale)
        _set_color((r, g, b))
        phase = (phase + 4) % 200
        stop.wait(0.05)


def _run_breathe_custom(
    stop: threading.Event,
    base: Tuple[int, int, int],
    sleep_s: float,
) -> None:
    _init_led()
    phase = 0
    while not stop.is_set():
        level = phase if phase <= 100 else 200 - phase
        scale = max(5, level) / 100.0
        r = int(base[0] * scale)
        g = int(base[1] * scale)
        b = int(base[2] * scale)
        _set_color((r, g, b))
        phase = (phase + 6) % 200
        stop.wait(sleep_s)


def _run_blink(stop: threading.Event, base: Tuple[int, int, int]) -> None:
    _init_led()
    on = True
    while not stop.is_set():
        _set_color(base if on else (0, 0, 0))
        on = not on
        stop.wait(0.15)


def _wheel(pos: int) -> Tuple[int, int, int]:
    try:
        if _led is not None and hasattr(_led, 'wheel'):
            r, g, b = _led.wheel(pos)
            return int(r), int(g), int(b)
    except Exception:
        pass

    if pos < 0 or pos > 255:
        return 0, 0, 0
    if pos < 85:
        return pos * 3, 255 - pos * 3, 0
    if pos < 170:
        pos -= 85
        return 255 - pos * 3, 0, pos * 3
    pos -= 170
    return 0, pos * 3, 255 - pos * 3


def _run_rainbow(stop: threading.Event) -> None:
    _init_led()
    strip = _get_strip()
    count = _strip_count(strip)
    if strip is None or count <= 0:
        return
    j = 0
    while not stop.is_set():
        for i in range(count):
            if stop.is_set():
                break
            pos = (int(i * 256 / count) + j) & 255
            _set_pixel(strip, i, _wheel(pos))
        _show(strip)
        j = (j + 1) & 255
        stop.wait(0.05)


def _start_fx(
    mode: str,
    color: Optional[Tuple[int, int, int]] = None,
    sleep_s: Optional[float] = None,
) -> None:
    global _fx_thread, _fx_stop, _fx_mode, _fx_color
    with _lock:
        if (
            _fx_mode == mode
            and _fx_color == color
            and _fx_thread is not None
            and _fx_thread.is_alive()
        ):
            return
    _stop_fx()
    if mode == 'off':
        _set_color((0, 0, 0))
        return
    if mode in ('breathe', 'blink') and color is None:
        return
    stop = threading.Event()
    if mode == 'breathe':
        if sleep_s is None:
            runner = (_run_breathe, (stop, color))
        else:
            runner = (_run_breathe_custom, (stop, color, float(sleep_s)))
    elif mode == 'blink':
        runner = (_run_blink, (stop, color))
    elif mode == 'rainbow':
        runner = (_run_rainbow, (stop,))
    else:
        return
    thread = threading.Thread(
        target=_wrap_runner,
        args=(stop, runner[0], runner[1]),
        daemon=True,
    )
    with _lock:
        _fx_stop = stop
        _fx_thread = thread
        _fx_mode = mode
        _fx_color = color
    thread.start()


def set_led_green() -> None:
    set_led_search()


def set_led_red() -> None:
    set_led_track()


def set_led_yellow() -> None:
    set_led_avoid()


def set_led_off() -> None:
    _start_fx('off')


def set_led_search() -> None:
    _start_fx('breathe', (0, 255, 0), sleep_s=0.05)


def set_led_avoid() -> None:
    _start_fx('blink', (255, 255, 0))


def set_led_detect() -> None:
    _start_fx('breathe', (0, 0, 255), sleep_s=0.05)


def set_led_track() -> None:
    _start_fx('breathe', (255, 0, 0), sleep_s=0.02)


def set_led_success() -> None:
    _start_fx('rainbow')


def _try_buzzer_beep(buzzer, on_time: float, off_time: float, n: int) -> bool:
    if not hasattr(buzzer, 'beep'):
        return False
    try:
        buzzer.beep(on_time=on_time, off_time=off_time, n=n, background=False)
        return True
    except TypeError:
        pass
    try:
        buzzer.beep(on_time=on_time, off_time=off_time, n=n)
        return True
    except TypeError:
        pass
    try:
        buzzer.beep(on_time, off_time, n, False)
        return True
    except TypeError:
        pass
    try:
        buzzer.beep(on_time, off_time, n)
        return True
    except Exception:
        return False


def beep_success() -> None:
    with _lock:
        resume_rainbow = _fx_mode == 'rainbow'
    _stop_fx()
    _init_buzzer()
    if not _buzzer_ok or _buzzer is None:
        return
    try:
        _buzzer.on()
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            time.sleep(0.05)
    except Exception:
        pass
    finally:
        _close_buzzer()
        if resume_rainbow:
            set_led_success()


def beep_fail() -> None:
    _stop_fx()
    _init_buzzer()
    if not _buzzer_ok or _buzzer is None:
        return
    try:
        if not _try_buzzer_beep(_buzzer, on_time=0.12, off_time=0.12, n=3):
            for _ in range(3):
                _buzzer.on()
                time.sleep(0.12)
                _buzzer.off()
                time.sleep(0.12)
    except Exception:
        pass
    finally:
        _close_buzzer()


def _cleanup() -> None:
    try:
        set_led_off()
    except Exception:
        pass
    _close_buzzer()


atexit.register(_cleanup)
