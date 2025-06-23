"""
wheel_to_twist_gui.py — Twist-PID GUI (полная версия, с одометрией)
───────────────────────────────────────────────────────────────────
• линейная v (мм/с) и угловая ω (°/с) ↔ скорости колёс
• Enc L/R, фактические vL/vR, v_fact, ω_fact, X-Y-θ
• Reset Enc, Reset Odom, Stop 0
• PID-слайдеры (Kp Ki Kd Kff)
• живой график «цель/факт» (20 с окно)
"""
import tkinter as tk
from tkinter import ttk
import requests, time, math
import matplotlib; matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ─── настройки ───
ESP      = "http://192.168.0.152"
POLL_S   = 0.10
WIN_SEC  = 20
BASE_MM  = 96.0                      # база робота, мм
# ─────────────────

def http(path): return requests.get(ESP + path, timeout=0.35)

# ---- преобразования twist ↔ wheels ----
def wheels_from_twist(v_mm, w_deg):
    ω = w_deg * math.pi / 180
    return int(v_mm - ω*BASE_MM/2), int(v_mm + ω*BASE_MM/2)

def twist_from_wheels(vL, vR):
    v  = 0.5*(vL+vR)
    ωd = (vR-vL) / BASE_MM * 180 / math.pi
    return v, ωd

# =======================================
class TwistGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 • Twist-PID GUI")
        self.resizable(False, False)

        # ---------- текстовые поля ----------
        self.encL=tk.StringVar(self,"—"); self.encR=tk.StringVar(self,"—")
        self.vL  =tk.StringVar(self,"—"); self.vR  =tk.StringVar(self,"—")
        self.vAct=tk.StringVar(self,"—"); self.wAct=tk.StringVar(self,"—")
        self.xPos=tk.StringVar(self,"—"); self.yPos=tk.StringVar(self,"—"); self.thPos=tk.StringVar(self,"—")

        for i,(lbl,var) in enumerate((
                ("Enc L",self.encL),("Enc R",self.encR),
                ("vL mm/s",self.vL),("vR mm/s",self.vR))):
            ttk.Label(self,text=lbl).grid(row=i,column=0,sticky="e",padx=6)
            ttk.Label(self,textvariable=var,width=10
                      ).grid(row=i,column=1,sticky="w")

        ttk.Label(self,text="v fact mm/s").grid(row=0,column=2,sticky="e")
        ttk.Label(self,textvariable=self.vAct,width=9
                  ).grid(row=0,column=3,sticky="w")
        ttk.Label(self,text="ω fact °/s").grid(row=1,column=2,sticky="e")
        ttk.Label(self,textvariable=self.wAct,width=9
                  ).grid(row=1,column=3,sticky="w")

        ttk.Label(self,text="X [m]").grid(row=2,column=2,sticky="e")
        ttk.Label(self,textvariable=self.xPos,width=9
                  ).grid(row=2,column=3,sticky="w")
        ttk.Label(self,text="Y [m]").grid(row=3,column=2,sticky="e")
        ttk.Label(self,textvariable=self.yPos,width=9
                  ).grid(row=3,column=3,sticky="w")
        ttk.Label(self,text="θ [rad]").grid(row=2,column=4,sticky="e")
        ttk.Label(self,textvariable=self.thPos,width=9
                  ).grid(row=2,column=5,sticky="w")

        # кнопки reset
        ttk.Button(self,text="Reset Enc",command=self.reset_enc
                   ).grid(row=0,column=4,rowspan=2,padx=6,sticky="ns")
        ttk.Button(self,text="Reset Odom",command=self.reset_odom
                   ).grid(row=2,column=6,rowspan=2,padx=6,sticky="ns")

        #ttk.Separator(self,orient="h").grid(row=5,columnspan=7,sticky="ew",pady=4)

        # ---------- PID слайдеры ----------
        self.pid={}
        for r,(name,mx,res) in enumerate((("Kp",10,0.1),("Ki",10,0.1),
                                          ("Kd",1,0.005),("Kff",1,0.05))):
            ttk.Label(self,text=name).grid(row=r,column=7,sticky="e",padx=6)
            if name=="Kp":
                var=tk.DoubleVar(self,1); self.pid[name]=var
            elif name=="Ki":
                var=tk.DoubleVar(self,0.8); self.pid[name]=var
            elif name=="Kd":
                var=tk.DoubleVar(self,0.02); self.pid[name]=var
            elif name=="Kff":
                var=tk.DoubleVar(self,0.25); self.pid[name]=var
        
            tk.Scale(self,from_=0,to=mx,resolution=res,orient="h",length=150,
                     variable=var,command=lambda *_:self.send_coeff()
                     ).grid(row=r,column=8,padx=2)

        # ---------- target wheel ----------
        self.wL=tk.IntVar(self,0); self.wR=tk.IntVar(self,0)
        ttk.Label(self,text="Wheel target (mm/s)").grid(row=5,column=7,columnspan=2)
        tk.Scale(self,from_=-200,to=200,orient="h",length=150,
                 variable=self.wL,command=self.on_wheel_change
                 ).grid(row=6,column=7)
        tk.Scale(self,from_=-200,to=200,orient="h",length=150,
                 variable=self.wR,command=self.on_wheel_change
                 ).grid(row=6,column=8)

        # ---------- target twist ----------
        self.lin=tk.IntVar(self,0); self.ang=tk.IntVar(self,0)
        ttk.Label(self,text="Linear v mm/s").grid(row=5,column=0,columnspan=2,sticky="e")
        tk.Scale(self,from_=-200,to=200,orient="h",length=180,
                 variable=self.lin,command=self.on_twist_change
                 ).grid(row=5,column=2,columnspan=2)
        ttk.Label(self,text="Angular °/s").grid(row=6,column=0,columnspan=2,sticky="e")
        tk.Scale(self,from_=60,to=-60,orient="h",length=180,
                 variable=self.ang,command=self.on_twist_change
                 ).grid(row=6,column=2,columnspan=2)
        ttk.Button(self,text="Stop 0",command=self.stop_all
                   ).grid(row=6,column=4,columnspan=2,padx=6)

        # ---------- график ----------
        fig=Figure(figsize=(11,3),dpi=100); self.ax=fig.add_subplot(111)
        self.ax.set_xlabel("time, s"); self.ax.set_ylabel("mm/s")
        self.lML,=self.ax.plot([],[],"C0-",  label="meas L")
        self.lTL,=self.ax.plot([],[],"C0--", label="tgt  L")
        self.lMR,=self.ax.plot([],[],"C1-",  label="meas R")
        self.lTR,=self.ax.plot([],[],"C1--", label="tgt  R")
        self.ax.legend(loc="upper left")
        FigureCanvasTkAgg(fig,master=self
            ).get_tk_widget().grid(row=7,column=0,columnspan=9,pady=8)

        # ---------- буферы графика ----------
        self.t0 = time.time()
        self.t   = []
        self.meL = []; self.meR = []
        self.tgL = []; self.tgR = []

        self.after(int(POLL_S*1000), self.poll)

    # ---------- REST ----------
    def send_coeff(self):
        qs="/setCoeff?"+"&".join(f"{k.lower()}={v.get():.3f}" for k,v in self.pid.items())
        try:http(qs)
        except:pass
    def send_speed(self,l,r):
        try:http(f"/setSpeed?l={int(l)}&r={int(r)}")
        except:pass
    def reset_enc(self):
        http("/resetEnc")
        self.t.clear(); self.meL.clear(); self.meR.clear(); self.tgL.clear(); self.tgR.clear()
    def reset_odom(self):
        http("/resetOdom")

    # ---------- callbacks ----------
    def on_wheel_change(self,*_):
        l=self.wL.get(); r=self.wR.get()
        v,w = twist_from_wheels(l,r)
        self.lin.set(int(v)); self.ang.set(int(w))
        self.send_speed(l,r)
    def on_twist_change(self,*_):
        v=self.lin.get(); w=self.ang.get()
        l,r = wheels_from_twist(v,w)
        self.wL.set(l); self.wR.set(r)
        self.send_speed(l,r)
    def stop_all(self):
        self.lin.set(0); self.ang.set(0); self.on_twist_change()

    # ---------- polling ----------
    def poll(self):
        try:
            d=http("/state").json()
            # текст
            self.encL.set(d["enc"]["left"]); self.encR.set(d["enc"]["right"])
            self.vL.set(f"{d['speed']['left']:.0f}"); self.vR.set(f"{d['speed']['right']:.0f}")
            vF,wF=twist_from_wheels(d['speed']['left'],d['speed']['right'])
            self.vAct.set(f"{vF:.0f}"); self.wAct.set(f"{wF:.0f}")
            self.xPos.set(f"{d['odom']['x']:.3f}"); self.yPos.set(f"{d['odom']['y']:.3f}")
            self.thPos.set(f"{d['odom']['th']:.3f}")

            # график
            t=time.time()-self.t0
            self.t.append(t); self.meL.append(d['speed']['left']); self.meR.append(d['speed']['right'])
            self.tgL.append(d['target']['left']); self.tgR.append(d['target']['right'])
            while self.t and t-self.t[0]>WIN_SEC:
                for buf in (self.t,self.meL,self.meR,self.tgL,self.tgR): buf.pop(0)
            self.lML.set_data(self.t,self.meL); self.lMR.set_data(self.t,self.meR)
            self.lTL.set_data(self.t,self.tgL); self.lTR.set_data(self.t,self.tgR)
            self.ax.relim(); self.ax.autoscale_view(); self.ax.set_xlim(max(0,t-WIN_SEC),t)
            self.ax.figure.canvas.draw_idle()
        except Exception:
            pass
        self.after(int(POLL_S*1000), self.poll)

# ——— run ———
if __name__=="__main__":
    TwistGUI().mainloop()
