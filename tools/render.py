#!/usr/bin/env python3
"""Vídeo de uma partida (vista de cima) a partir dos quadros gravados pelo tools/lockstep.py (*_frames.json, 10 Hz).

Mostra o campo, os robôs (com a direção para onde olham), a bola ou as bandeiras, o placar, o tempo e as mensagens
do juiz. No rouba-bandeira, um contorno amarelo marca o robô preso.

Uso (Python com Pillow e o ffmpeg no PATH):
  python3 tools/render.py exercicios/registros/final/soccer_final_x_v2_azul_p1_frames.json \
      exercicios/videos/futebol_final_x_v2.mp4 [--start 0] [--seconds 90] [--speed 1]
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 960, 720
FPS = 10
BLUE, RED, ORANGE = (40, 100, 230), (220, 50, 40), (245, 140, 20)
HERO_R = 0.037


def font(size, bold=False):
    for path in ("/usr/share/fonts/TTF/DejaVuSans%s.ttf" % ("-Bold" if bold else ""),
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else "")):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


ARENAS = {  # meia-largura, meia-altura (m) da área desenhada
    "sumo": (0.5, 0.5),
    "ctf": (0.85, 0.85),
    "soccer": (1.02, 0.65),
}


def colors(scenario):
    if scenario == "sumo":
        return [(50, 140, 230), (240, 150, 40)]
    if scenario == "ctf":
        return [BLUE, BLUE, RED, RED]
    return [BLUE] * 3 + [RED] * 3


def scoreboard(scenario, g):
    if scenario == "sumo":
        return "hero_0  %d  x  %d  hero_1     empates: %d" % (g["wins"][0], g["wins"][1], g["draws"])
    return "AZUL  %d  x  %d  VERMELHO" % (g["score"][0], g["score"][1])


def draw_frame(scenario, fr, title):
    img = Image.new("RGB", (W, H), (235, 236, 240))
    d = ImageDraw.Draw(img)
    hx, hy = ARENAS[scenario]
    top = 84
    scale = min((W - 40) / (2 * hx), (H - top - 60) / (2 * hy))
    cx, cy = W / 2, top + (H - top - 60) / 2

    def P(x, y):
        return cx + x * scale, cy - y * scale

    def rect(x0, y0, x1, y1, **kw):
        a, b = P(x0, y1), P(x1, y0)
        d.rectangle([a[0], a[1], b[0], b[1]], **kw)

    def circle(x, y, r, **kw):
        a, b = P(x - r, y + r), P(x + r, y - r)
        d.ellipse([a[0], a[1], b[0], b[1]], **kw)

    if scenario == "sumo":
        circle(0, 0, 0.412, fill=(245, 245, 245))
        circle(0, 0, 0.40, fill=(35, 35, 40))
    elif scenario == "ctf":
        rect(-0.8, -0.8, 0.8, 0.8, fill=(225, 228, 238), outline=(50, 52, 65), width=4)
        rect(-0.8, -0.8, 0, 0.8, fill=(215, 222, 245))
        rect(0, -0.8, 0.8, 0.8, fill=(245, 220, 218))
        d.line([P(0, -0.8), P(0, 0.8)], fill=(110, 115, 140), width=3)
        for bx, c in ((-0.62, BLUE), (0.62, RED)):
            circle(bx, 0, 0.14, outline=c, width=3)
    else:
        rect(-0.9, -0.6, 0.9, 0.6, fill=(40, 115, 55), outline=(245, 245, 245), width=3)
        d.line([P(0, -0.6), P(0, 0.6)], fill=(245, 245, 245), width=2)
        circle(0, 0, 0.15, outline=(245, 245, 245), width=2)
        rect(-1.0, -0.2, -0.9, 0.2, fill=(30, 60, 140))
        rect(0.9, -0.2, 1.0, 0.2, fill=(140, 30, 30))

    if scenario == "ctf" and fr.get("f"):
        for (fx, fy), c in zip(fr["f"], (BLUE, RED)):
            a = P(fx, fy)
            d.line([a, (a[0], a[1] - 26)], fill=(30, 30, 30), width=3)
            d.polygon([(a[0], a[1] - 26), (a[0] + 18, a[1] - 20), (a[0], a[1] - 14)], fill=c)

    for k, (x, y, yaw, held) in enumerate(fr["r"]):
        c = colors(scenario)[k]
        if held:
            circle(x, y, HERO_R + 0.014, fill=(255, 215, 0))
        circle(x, y, HERO_R, fill=c, outline=(20, 20, 20), width=2)
        a = P(x, y)
        b = P(x + HERO_R * 1.25 * math.cos(yaw), y + HERO_R * 1.25 * math.sin(yaw))
        d.line([a, b], fill=(255, 255, 255), width=3)
        d.text((a[0], a[1]), str(k), fill=(255, 255, 255), font=font(13, True), anchor="mm")

    if fr.get("b"):
        circle(fr["b"][0], fr["b"][1], 0.0214, fill=ORANGE, outline=(20, 20, 20))

    d.rectangle([0, 0, W, top - 10], fill=(28, 30, 38))
    d.text((20, 10), title, fill=(200, 205, 220), font=font(15))
    d.text((W / 2, 54), scoreboard(scenario, fr["g"]), fill=(255, 255, 255), font=font(22, True), anchor="mm")
    d.text((W - 20, 10), "t = %5.1f s" % fr["t"], fill=(200, 205, 220), font=font(16), anchor="ra")
    if fr.get("m"):
        d.rectangle([0, H - 46, W, H], fill=(255, 244, 214))
        d.text((W / 2, H - 23), fr["m"], fill=(150, 20, 35), font=font(22, True), anchor="mm")
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames")
    ap.add_argument("out")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--speed", type=float, default=1.0, help="2 = duas vezes mais rápido que o tempo real")
    ap.add_argument("--title", default="")
    a = ap.parse_args()

    data = json.loads(Path(a.frames).read_text())
    scenario = data["scenario"]
    frames = [f for f in data["frames"] if a.start <= f["t"] < a.start + a.seconds]
    title = a.title or Path(a.frames).name.replace("_frames.json", "")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (W, H),
           "-r", str(FPS * a.speed), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26",
           "-movflags", "+faststart", a.out]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for fr in frames:
        proc.stdin.write(draw_frame(scenario, fr, title).tobytes())
    proc.stdin.close()
    proc.wait()
    print("%s: %d quadros (%.0f s de jogo)" % (a.out, len(frames), len(frames) / FPS))


if __name__ == "__main__":
    main()
