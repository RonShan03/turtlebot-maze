"""Render the submission Markdown to PDF and a self-contained HTML document.

Optional dependencies: pip install reportlab markdown
"""
import base64
import html
from pathlib import Path
import re


def main():
    import markdown
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_LEFT
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle, KeepTogether

    root=Path(__file__).resolve().parents[1]
    source=(root/'REPORT.md').read_text(encoding='utf-8-sig')
    body=markdown.markdown(source,extensions=['tables'])
    def embed(match):
        path=root/match[1]
        mime='image/png' if path.suffix=='.png' else 'image/jpeg'
        return 'src="data:'+mime+';base64,'+base64.b64encode(path.read_bytes()).decode()+'"'
    body=re.sub(r'src="([^"]+)"',embed,body)
    (root/'REPORT.html').write_text('<!doctype html><meta charset="utf-8"><title>DS685 Object Detection</title>'
        '<style>body{max-width:1050px;margin:40px auto;font:16px/1.6 system-ui;color:#182433}'
        'img{max-width:100%;height:auto}table{border-collapse:collapse;width:100%;font-size:14px}'
        'td,th{border:1px solid #ccd3dc;padding:7px;text-align:left}th{background:#e8edf5}'
        'h1,h2{color:#193958}a{color:#155c9c}@media print{body{margin:0}h2{break-after:avoid}}'
        '</style>'+body,encoding='utf-8')

    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='ReportBody',fontName='Helvetica',fontSize=9.5,leading=14,
                              spaceAfter=9,alignment=TA_LEFT))
    styles.add(ParagraphStyle(name='Cell',fontSize=7,leading=9))
    styles['Heading1'].fontSize=16;styles['Heading1'].leading=20
    styles['Heading2'].fontSize=12;styles['Heading2'].leading=16
    def inline(text):
        text=html.escape(text)
        text=re.sub(r'`([^`]+)`',r'<font name="Courier">\1</font>',text)
        text=re.sub(r'\[([^\]]+)\]\((https?://[^)]+)\)',r'<link href="\2" color="#155c9c">\1</link>',text)
        return text
    flow=[];lines=source.splitlines();i=0
    while i<len(lines):
        line=lines[i].strip();i+=1
        if not line:continue
        if line.startswith('|'):
            rows=[line]
            while i<len(lines) and lines[i].strip().startswith('|'):
                rows.append(lines[i].strip());i+=1
            cells=[[Paragraph(inline(v.strip()),styles['Cell']) for v in r.strip('|').split('|')]
                   for r in rows if not re.fullmatch(r'[|:\s-]+',r)]
            widths=[515/len(cells[0])]*len(cells[0])
            if 'Tie breaker' in rows[0]:widths=[45,95,65,65,60,55,65,65]
            table=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
            table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e8edf5')),
                ('GRID',(0,0),(-1,-1),.4,colors.HexColor('#ccd3dc')),('VALIGN',(0,0),(-1,-1),'TOP'),
                ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]))
            flow.append(KeepTogether([table,Spacer(1,12)]));continue
        match=re.fullmatch(r'!\[([^\]]*)\]\(([^)]+)\)',line)
        if match:
            im=Image(str(root/match[2]));scale=min(515/im.imageWidth,310/im.imageHeight)
            im.drawWidth=im.imageWidth*scale;im.drawHeight=im.imageHeight*scale
            im.hAlign='LEFT';flow.extend([im,Spacer(1,6)]);continue
        if line.startswith('# '):flow.append(Paragraph(inline(line[2:]),styles['Title']));continue
        if line.startswith('## '):flow.append(Paragraph(inline(line[3:]),styles['Heading2']));continue
        if line.startswith('- '):
            flow.append(Paragraph(inline(line[2:]),styles['ReportBody'],bulletText='-'));continue
        paragraph=[line]
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','|','![','- ')):
            paragraph.append(lines[i].strip());i+=1
        flow.append(Paragraph(inline(' '.join(paragraph)),styles['ReportBody']))
    def footer(canvas,doc):
        canvas.setFont('Helvetica',8);canvas.setFillColor(colors.HexColor('#596675'))
        canvas.drawString(40,25,'DS685 | Rohan Shanbhag | house03')
        canvas.drawRightString(555,25,str(doc.page))
    doc=SimpleDocTemplate(str(root/'REPORT.pdf'),pagesize=(595,842),leftMargin=40,rightMargin=40,
        topMargin=35,bottomMargin=40,title='DS685 RGB-D Object Detection and Semantic Memory',author='Rohan Shanbhag')
    doc.build(flow,onFirstPage=footer,onLaterPages=footer)
    print('Rendered REPORT.pdf and self-contained REPORT.html')


if __name__=='__main__':main()
