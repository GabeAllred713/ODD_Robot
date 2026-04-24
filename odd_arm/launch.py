import customtkinter as ctk
import configparser
import subprocess as sp
import select
import os
import signal

from pathlib import Path


class WarningDialog(ctk.CTkToplevel):
    def __init__(self, parent=None, title="", text=""):
        super().__init__(parent)
        self.geometry("230x100")
        self.title(title)
        
        self.box = ctk.CTkLabel(self, text=text, wraplength=200, text_color="red")
        self.box.pack(padx=20, pady=0)
        
        self.button = ctk.CTkButton(self, text="Ok", command=self.destroy)
        self.button.pack(padx=20, pady=0)


class ODDARMLaunch(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.geometry("600x400")
        self.title("ODD ARM Launchpad")
        self.grid_rowconfigure(5, weight=1)
        self.columnconfigure(1, weight=1)
        
        self.config = configparser.ConfigParser()
        self.config.read("config.ini")
        
        self.odd_button = ctk.CTkButton(self, text="ODD", width=200, command=self.toggle_odd)
        self.odd_button.grid(row=0, column=0, padx=20, pady=5)
        
        self.arm_button = ctk.CTkButton(self, text="ARM", width=200, command=self.toggle_arm)
        self.arm_button.grid(row=1, column=0, padx=20, pady=5)

        self.arm_rviz_button = ctk.CTkButton(self, text="ARM RViz", width=200, command=self.toggle_arm_rviz)
        self.arm_rviz_button.grid(row=2, column=0, padx=20, pady=5)
        
        self.cv_button = ctk.CTkButton(self, text="Computer Vision", width=200, command=self.toggle_cv)
        self.cv_button.grid(row=3, column=0, padx=20, pady=5)
        
        self.log_label = ctk.CTkLabel(self, text="Console Log")
        self.log_label.grid(row=4, column=0, padx=0, pady=0)
        
        self.combined_log = ctk.CTkTextbox(master=self, width=600-40)
        self.combined_log.grid(row=5, column=0, sticky="nsew", padx=20, pady=0)
        self.combined_log.configure(state="disabled")
        
        self.odd_process = None
        self.cv_process = None
        self.arm_process = None
        self.arm_rviz_process = None
        
        self.running = True
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.process_log()


    def add_to_log(self, text):
        self.combined_log.configure(state="normal")
        self.combined_log.insert("0.0", text)
        self.combined_log.delete("1000.0", "end")
        self.combined_log.configure(state="disabled")
        
        if "nano" in text and "out of memory" in text:
            WarningDialog(self, title="No Memory", text="Not enough memory to open NanoOWL. Try closing programs and clearing cache with jtop.")
            self._close_process("cv")


    def process_log(self):
        self._process_log("odd")
        self._process_log("arm")
        self._process_log("arm_rviz")
        self._process_log("cv")
        
        if self.running:
            self.after(10, self.process_log)
    
    
    def _process_log(self, name, ):
        process = getattr(self, name + "_process")
        if process is None:
            return

        if process.poll() is not None:
            self._close_process(name)
            return

        readable, _, _ = select.select([process.stdout, process.stderr], [], [], 0)

        for pipe in readable:
            try:
                raw_data = os.read(pipe.fileno(), 4096)
                if not raw_data:
                    continue

                text = raw_data.decode('utf-8', errors='replace')
                self.add_to_log(text)
            except Exception as e:
                self.add_to_log(f"Error reading pipe: {e}\n")


    def _close_process(self, name):
        process = getattr(self, name + "_process")
        if process is None:
            return


        process.poll()
        if process.returncode is not None:
            self._close_process_cleanup(name, process)
            return


        pgid = os.getpgid(process.pid)
        
        try:
            os.killpg(pgid, signal.SIGINT)
            process.communicate(timeout=5)
        except (sp.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(pgid, signal.SIGKILL)
                process.wait()
            except ProcessLookupError:
                self.add_to_log("ProcessLookupError!")
        finally:
            if process.stdout: process.stdout.close()
            if process.stderr: process.stderr.close()

        self._close_process_cleanup(name, process)


    def _close_process_cleanup(self, name, process):
        self.add_to_log(f"{name} exited with return code {process.returncode}\n")

        if name == "odd":
            self.odd_button.configure(text="ODD")
        elif name == "arm":
            self.arm_button.configure(text="ARM")
        elif name == "arm_rviz":
            self.arm_rviz_button.configure(text="ARM RViz")
        elif name == "cv":
            self.cv_button.configure(text="Computer Vision")
            sp.run("docker kill --signal SIGTERM odd_arm_nanoowl", shell=True)  # Docker is misbehaving and I'm too tired to figure out a better way
        

        setattr(self, name + "_process", None)


    def _launch_process(self, name):
        if getattr(self, name + "_process") is not None:
            self.add_to_log(f"{name} is already running!\n")
            return

        self.add_to_log(f"Launching {name}\n")

        script_path = Path(self.config.get("paths", name)).resolve()
        setattr(self, name + "_process", sp.Popen(
            [str(script_path)],
            stdout=sp.PIPE,
            stderr=sp.PIPE,
            cwd=script_path.parent,
            start_new_session=True
        ))
    
    
    def toggle_odd(self):
        if self.odd_process is None:
            self._launch_process("odd")
            self.odd_button.configure(text="Close ODD")
        else:
            self._close_process("odd")
        
        
    def toggle_arm(self):
        if self.arm_process is None:
            self._launch_process("arm")
            self.arm_button.configure(text="Close ARM")
        else:
            self._close_process("arm")

    def toggle_arm_rviz(self):
        if self.arm_rviz_process is None:
            self._launch_process("arm_rviz")
            self.arm_rviz_button.configure(text="Close ARM RViz")
        else:
            self._close_process("arm_rviz")
    
    def toggle_cv(self):
        if self.cv_process is None:
            self._launch_process("cv")
            self.cv_button.configure(text="Close Computer Vision")
        else:
            self._close_process("cv")
    
    
    def on_close(self):
        self.running = False
    
        self._close_process("odd")
        self._close_process("cv")
        self._close_process("arm")
        self._close_process("arm_rviz")
        
        self.destroy()


app = ODDARMLaunch()
app.mainloop()
