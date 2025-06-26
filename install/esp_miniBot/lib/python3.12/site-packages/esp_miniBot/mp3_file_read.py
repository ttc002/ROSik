import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import subprocess
import threading
import time
import os

AUDIO_FOLDER = '/home/rx/Downloads'
CHUNK_SIZE = 4096

def get_track_duration(path):
    try:
        result = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "default=noprint_wrappers=1:nokey=1", path],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return float(result.stdout.strip())
    except Exception:
        return 0.0

class AudioInterface(Node):
    def __init__(self):
        super().__init__('audio_interface_node')
        self.audio_pub = self.create_publisher(CompressedImage, 'audio_bytes', 10)
        self.track_list = [f for f in os.listdir(AUDIO_FOLDER) if f.endswith('.mp3') or f.endswith('.wav')]
        self.current_track_index = 0
        self.playing = False
        self.volume = 1.0
        self.position = 0.0
        self.duration = 0.0
        self.seek_to = None
        self.restart = False
        self.lock = threading.Lock()
        self.process = None
        threading.Thread(target=self.audio_loop, daemon=True).start()
        #self.declare_parameter("folder","/home/rx/Downloads")
        #AUDIO_FOLDER = self.get_parameter("folder").value
    def stop_ffmpeg(self):
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=1)
            except Exception:
                self.process.kill()
            self.process = None

    def audio_loop(self):
        while True:
            with self.lock:
                playing = self.playing
                track_list = self.track_list
            if not playing or not track_list:
                self.stop_ffmpeg()
                time.sleep(0.1)
                continue
            with self.lock:
                track = os.path.join(AUDIO_FOLDER, self.track_list[self.current_track_index])
                self.duration = get_track_duration(track)
                pos = self.position if self.seek_to is None else self.seek_to
                vol = self.volume
                self.position = pos
                seek = self.seek_to
                restart = self.restart
                self.seek_to = None
                self.restart = False
            self.stop_ffmpeg()
            cmd = [
                "ffmpeg", "-ss", str(pos), "-i", track,
                "-filter:a", f"volume={vol}",
                "-f", "s16le", "-acodec", "pcm_s16le", "-ac", "2", "-ar", "22050", "-"
            ]
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            chunk_dur = CHUNK_SIZE / (2 * 2 * 22050)
            while True:
                with self.lock:
                    if not self.playing or self.seek_to is not None or self.restart:
                        break
                chunk = self.process.stdout.read(CHUNK_SIZE)
                if not chunk:
                    break
                msg = CompressedImage()
                msg.format = 's16le'
                msg.data = chunk
                self.audio_pub.publish(msg)
                with self.lock:
                    self.position += chunk_dur
                    if self.position > self.duration:
                        self.position = self.duration
                time.sleep(chunk_dur)
            with self.lock:
                if self.seek_to is not None or self.restart:
                    continue
                if self.position >= self.duration:
                    self.position = 0.0
                    self.playing = False
            self.stop_ffmpeg()

    def play(self):
        with self.lock:
            self.playing = True
            self.restart = True

    def pause(self):
        with self.lock:
            self.playing = False
            self.stop_ffmpeg()

    def next_track(self):
        with self.lock:
            self.current_track_index = (self.current_track_index + 1) % len(self.track_list)
            self.position = 0.0
            self.playing = True
            self.restart = True

    def prev_track(self):
        with self.lock:
            self.current_track_index = (self.current_track_index - 1) % len(self.track_list)
            self.position = 0.0
            self.playing = True
            self.restart = True

    def change_volume(self, delta):
        with self.lock:
            self.volume = max(0.0, min(2.0, self.volume + delta))
            if self.playing:
                self.restart = True

    def seek(self, seconds):
        with self.lock:
            new_pos = max(0.0, min(self.duration, self.position + seconds))
            self.seek_to = new_pos

    def set_position(self, pos):
        with self.lock:
            new_pos = max(0.0, min(self.duration, pos))
            self.seek_to = new_pos

def format_time(seconds):
    m = int(seconds // 60)
    s = int(seconds % 60)
    return f"{m}:{s:02d}"

def main():
    rclpy.init()
    node = AudioInterface()
    pygame.init()
    screen = pygame.display.set_mode((500, 180))
    pygame.display.set_caption("Audio Player")
    font = pygame.font.SysFont(None, 36)
    small_font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()
    slider_rect = pygame.Rect(20, 120, 460, 20)
    volume_rect = pygame.Rect(20, 160, 200, 10)
    dragging_slider = False
    dragging_volume = False

    running = True
    node.play()
    while running:
        screen.fill((30, 30, 30))
        if node.track_list:
            text = font.render(node.track_list[node.current_track_index], True, (200, 200, 200))
            screen.blit(text, (20, 20))
        with node.lock:
            pos = node.position
            dur = node.duration
            vol = node.volume
            playing = node.playing
        time_text = small_font.render(f"{format_time(pos)} / {format_time(dur)}", True, (180, 180, 180))
        screen.blit(time_text, (20, 60))
        pygame.draw.rect(screen, (80, 80, 80), (20, 90, 60, 25))
        pygame.draw.rect(screen, (80, 80, 80), (90, 90, 60, 25))
        pygame.draw.rect(screen, (80, 80, 80), (160, 90, 60, 25))
        prev_label = small_font.render("Prev", True, (255,255,255))
        play_label = small_font.render("Pause" if playing else "Play", True, (255,255,255))
        next_label = small_font.render("Next", True, (255,255,255))
        screen.blit(prev_label, (30, 95))
        screen.blit(play_label, (100, 95))
        screen.blit(next_label, (170, 95))
        pygame.draw.rect(screen, (60, 60, 60), slider_rect)
        if dur > 0:
            slider_pos = int(slider_rect.x + (slider_rect.w - 10) * (pos / dur))
            pygame.draw.rect(screen, (100, 200, 255), (slider_pos, slider_rect.y, 10, slider_rect.h))
        pygame.draw.rect(screen, (60, 60, 60), volume_rect)
        vol_pos = int(volume_rect.x + (volume_rect.w - 10) * (vol / 2.0))
        pygame.draw.rect(screen, (100, 255, 100), (vol_pos, volume_rect.y, 10, volume_rect.h))
        vol_text = small_font.render(f"Volume: {vol:.2f}", True, (100, 255, 100))
        screen.blit(vol_text, (volume_rect.x + volume_rect.w + 10, volume_rect.y - 5))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if playing:
                        node.pause()
                    else:
                        node.play()
                elif event.key == pygame.K_RIGHT:
                    node.next_track()
                elif event.key == pygame.K_LEFT:
                    node.prev_track()
                elif event.key == pygame.K_UP:
                    node.change_volume(0.05)
                elif event.key == pygame.K_DOWN:
                    node.change_volume(-0.05)
                elif event.key == pygame.K_PAGEUP:
                    node.seek(5)
                elif event.key == pygame.K_PAGEDOWN:
                    node.seek(-5)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if pygame.Rect(20, 90, 60, 25).collidepoint(mx, my):
                    node.prev_track()
                elif pygame.Rect(90, 90, 60, 25).collidepoint(mx, my):
                    if playing:
                        node.pause()
                    else:
                        node.play()
                elif pygame.Rect(160, 90, 60, 25).collidepoint(mx, my):
                    node.next_track()
                elif slider_rect.collidepoint(mx, my):
                    dragging_slider = True
                    rel = (mx - slider_rect.x) / slider_rect.w
                    node.set_position(rel * dur if dur else 0)
                elif volume_rect.collidepoint(mx, my):
                    dragging_volume = True
                    rel = (mx - volume_rect.x) / volume_rect.w
                    node.change_volume((rel * 2.0) - vol)
            elif event.type == pygame.MOUSEBUTTONUP:
                dragging_slider = False
                dragging_volume = False
            elif event.type == pygame.MOUSEMOTION:
                mx, my = event.pos
                if dragging_slider and dur:
                    rel = (mx - slider_rect.x) / slider_rect.w
                    node.set_position(rel * dur)
                if dragging_volume:
                    rel = (mx - volume_rect.x) / volume_rect.w
                    node.change_volume((rel * 2.0) - vol)
        pygame.display.flip()
        clock.tick(30)
    node.pause()
    pygame.quit()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
