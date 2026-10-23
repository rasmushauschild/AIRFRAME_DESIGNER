"""Takeoff spin: reproduced in SITL, what stops it. Airy light document style."""
import sys
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, PageTemplate, Paragraph, Spacer, Table, TableStyle, Image,
                                KeepTogether, ListFlowable, ListItem, Flowable)

OUT, LOGO, FIG1, FIG2, FIGV = sys.argv[1:6]
SUP = "/System/Library/Fonts/Supplemental/"
pdfmetrics.registerFont(TTFont("Body", SUP + "Arial.ttf"))
pdfmetrics.registerFont(TTFont("Body-Bold", SUP + "Arial Bold.ttf"))
pdfmetrics.registerFontFamily("Body", normal="Body", bold="Body-Bold")
BLACK, GRAY, GREEN = colors.HexColor("#000000"), colors.HexColor("#6B6B6B"), colors.HexColor("#1E9255")
RULE, CALLOUT = colors.HexColor("#DDDDDD"), colors.HexColor("#F5F5F5")
body = ParagraphStyle("body", fontName="Body", fontSize=9.5, leading=13.5, textColor=BLACK, spaceAfter=5)
small = ParagraphStyle("small", parent=body, fontSize=8, leading=11, textColor=GRAY)
cell = ParagraphStyle("cell", parent=body, fontSize=8.5, leading=11, spaceAfter=0)
cellh = ParagraphStyle("cellh", parent=cell, fontName="Body-Bold", fontSize=7, leading=9)
right = ParagraphStyle("r", parent=cell, alignment=2); righth = ParagraphStyle("rh", parent=cellh, alignment=2)
h1 = ParagraphStyle("h1", fontName="Body", fontSize=19, leading=23, textColor=BLACK)
h2 = ParagraphStyle("h2", fontName="Body", fontSize=12, leading=15, textColor=BLACK, spaceBefore=12, spaceAfter=2)
lead = ParagraphStyle("lead", parent=body, fontSize=10.5, leading=15)

def head(bold, desc, style=h2):
    return Paragraph('<font name="Body-Bold">%s</font> <font color="#6B6B6B">%s</font>' % (bold.upper(), desc.upper()), style)

class Rule(Flowable):
    def __init__(self, space=6): super().__init__(); self.space = space
    def wrap(self, aw, ah): self.aw = aw; return aw, self.space
    def draw(self): self.canv.setStrokeColor(GREEN); self.canv.setLineWidth(0.75); self.canv.line(0, self.space - 2, self.aw, self.space - 2)

def callout(paras, width):
    t = Table([[paras]], colWidths=[width])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), CALLOUT), ("LINEBEFORE", (0, 0), (0, -1), 2, GREEN),
                           ("LEFTPADDING", (0, 0), (-1, -1), 12), ("RIGHTPADDING", (0, 0), (-1, -1), 12),
                           ("TOPPADDING", (0, 0), (-1, -1), 9), ("BOTTOMPADDING", (0, 0), (-1, -1), 5)]))
    return t

def bullets(items, numbered=False):
    return ListFlowable([ListItem(Paragraph(x, body), leftIndent=14) for x in items], bulletType="1" if numbered else "bullet",
                        start="1" if numbered else None, bulletFontName="Body-Bold" if numbered else "Body", bulletFontSize=9,
                        bulletColor=GREEN, leftIndent=14, bulletDedent=12)

def table(hdr, rows, widths, num_from=1):
    data = [[Paragraph(x.upper(), cellh if i < num_from else righth) for i, x in enumerate(hdr)]]
    data += [[Paragraph(x, cell if i < num_from else right) for i, x in enumerate(r)] for r in rows]
    t = Table(data, colWidths=widths, repeatRows=1)
    t.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, -1), 0.5, RULE), ("LINEBELOW", (0, 0), (-1, 0), 0.75, BLACK),
                           ("VALIGN", (0, 0), (-1, -1), "BOTTOM"), ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                           ("TOPPADDING", (0, 0), (-1, -1), 3.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5)]))
    return t

W, H = letter; M = 0.8 * inch; CW = W - 2 * M
def on_page(c, doc):
    c.saveState()
    c.drawImage(LOGO, W - M - 0.95 * inch, H - 0.6 * inch, width=0.95 * inch, height=0.24 * inch, preserveAspectRatio=True, mask="auto", anchor="ne")
    c.setFont("Body", 7.5); c.setFillColor(GRAY)
    c.drawString(M, 0.48 * inch, "SMALL_SCALE_V3 takeoff spin  |  Oct 20 flights (logs 409, 413, 414, 423, 424) and PX4 SITL")
    c.drawRightString(W - M, 0.48 * inch, "%d" % doc.page); c.restoreState()
doc = BaseDocTemplate(OUT, pagesize=letter, leftMargin=M, rightMargin=M, topMargin=0.9 * inch, bottomMargin=0.75 * inch,
                      title="Takeoff spin: reproduced in simulation", author="Stefanos Fragkoulis")
doc.addPageTemplates([PageTemplate("p", [Frame(M, 0.75 * inch, CW, H - 1.65 * inch, leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)], onPage=on_page)])

s = [head("Takeoff spin", "reproduced in simulation", h1), Spacer(1, 3),
     Paragraph("SMALL_SCALE_V3, five filmed flights on Oct 20 against PX4 SITL with the aircraft's own parameters. Prepared Oct 21, 2026.", small), Rule(space=9)]
s += [callout([head("Bottom line", "", ParagraphStyle("cb", parent=h2, spaceBefore=0)), Spacer(1, 2),
               Paragraph("The simulator reproduces the spin only when the fans turn the opposite way to what PX4 is told: clockwise seen from above, "
                         "while every <b>CA_ROTORn_KM</b> says counter-clockwise. It then matches the flights on peak rate, total turn and timing. "
                         "In the simulator the spin disappears when PX4 is given the real spin direction and the front pair's yaw stand-in is made "
                         "symmetric. The fans' real direction has not been checked on the aircraft yet.", lead)], CW)]

s += [head("Why it spins", "four things in a row", h2), Rule(),
      bullets(["<b>A yaw torque PX4 does not expect.</b> With the fans turning opposite to their declared spin, their reaction torque pushes the nose left "
               "(about 1.2 N·m in hover). It grows with thrust as the takeoff ramps up, so it appears as the aircraft gets light on its feet.",
               "<b>A stand-in that adds to it.</b> The front pair's yaw coefficients (+0.2696 / -0.2196) share a +0.025 offset that pushes the same way.",
               "<b>No heading hold during takeoff.</b> PX4's automatic takeoff sends no yaw target, so only the yaw-rate loop resists, and at P = 0.07 "
               "it asks for 0.12 of torque at -100 °/s.",
               "<b>A slow integrator does the stopping.</b> It starts at zero on the ground and grows at I = 0.035, reaching 0.27 after about 8 s. "
               "Heading hold then engages and the spin dies away. In hover the integrator sits at its 0.30 cap."], numbered=True)]

s += [KeepTogether([head("Simulation", "matched to the flights", h2), Rule(),
      Paragraph("All times from PX4's own <i>Takeoff detected</i>, measured the same way in both.", body),
      table(["", "Peak spin", "Peak at", "Turned in 8 s", "Integrator at 0.27", "Heading hold from"],
            [["Real flights (5)", "92 to 113 °/s", "+3.5 to +4.5 s", "-355° to -442°", "+7.8 to +8.2 s", "+7.2 to +9.6 s"],
             ["<b>Sim: fans clockwise, PX4 told counter-clockwise</b>", "<b>112 °/s</b>", "<b>+3.8 s</b>", "<b>-467°</b>", "<b>+7.4 s</b>", "<b>+7.2 s</b>"],
             ["Sim: fans as declared", "14 °/s", "+3.8 s", "-55°", "never", "+7.2 s"]],
            [2.2 * inch] + [(CW - 2.2 * inch) / 5] * 5)]), Spacer(1, 6), Image(FIG1, width=CW, height=CW * 3.6 / 8.6)]
s += [Paragraph("To make the simulated aircraft behave like the real one, only the simulation was changed: the board's 41 flight-relevant parameters "
                "(gyro filters 25/15 Hz, takeoff, estimator), a thrust curve matching THR_MDL_FAC 0.38 (exponent 1.33), 26 N per fan so it hovers "
                "at 0.69 (real 0.70 to 0.74), the front foot lowered so it rests at -5° pitch (real -5.5°), and Takeoff selected before arming "
                "as on the aircraft. The fans' torque coefficient was tried from 0.010 to 0.018: the spin reproduces across that range (101 to 132 °/s).", small)]
s += [KeepTogether([Spacer(1, 6), Image(FIGV, width=CW, height=CW * 0.142),
       Paragraph("IMG_7524 against log_413 (sound sync 0.81): seconds after takeoff, yaw rate, turn so far. The aircraft is still sliding on the floor "
                 "when the spin starts and is side-on at -111°.", small)])]

s += [KeepTogether([head("What stops it", "in simulation", h2), Rule(),
      table(["Change (fans clockwise in all runs)", "Peak spin", "Turned in 8 s", "Takeoff"],
            [["As flown", "112 °/s", "-467°", "completes"],
             ["Integrator cap 0.6 only", "112 °/s", "-467°", "completes"],
             ["Yaw gains P 0.2 / I 0.1 only", "60 °/s", "-170°", "completes"],
             ["PX4 told the real spin", "39 °/s", "-150°", "completes"],
             ["<b>PX4 told the real spin, front pair symmetric (0.2331 / -0.2561)</b>", "<b>5 °/s</b>", "<b>+7°</b>", "<b>completes</b>"],
             ["PX4 told the front pair's real tilt (AY ±0.5)*", "25 °/s", "-126°", "fails: motors pinned 81%"]],
            [3.3 * inch] + [(CW - 3.3 * inch) / 3] * 3),
      Paragraph("* Run before the thrust match (29 N fans); the takeoff failed there, so it was not repeated.", small),
      Spacer(1, 4), Image(FIG2, width=CW, height=CW * 3.6 / 8.6)])]
s += [Paragraph("With the spin and the symmetric front pair, no motor is pinned and all requested torque is delivered. It still holds if the fans "
                "turn out 35% stronger than assumed (14 °/s). Higher yaw gains add nothing once the model is right.", body)]

s += [head("Limits", "", h2), Rule(),
      bullets(["The fans' direction is inferred, not seen: the 60 fps video cannot show it. A check by eye from above, or a slow-motion clip, settles it.",
               "A same-sense sideways cant of every fan of about 1° would make a similar torque. If the fans do spin as declared, look there next.",
               "The simulated aircraft navigates on GPS instead of the H-flow (the flow-only estimate dropped out at liftoff in SITL). Yaw control is unaffected.",
               "SITL runs a v1.17-era PX4 build, the aircraft v1.18 beta (SYS_HAS_GPS stands in for SYS_HAS_NUM_GNSS). Landing was not evaluated."])]
s += [Paragraph("Nothing was changed on the aircraft or in the design. All changes above exist only in the simulation runs.", small)]
doc.build(s)
print("written", OUT)
