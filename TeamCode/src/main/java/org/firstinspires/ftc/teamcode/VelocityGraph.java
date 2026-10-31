package org.firstinspires.ftc.teamcode;

import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/**
 * Serves a live graph of one value. Connect a laptop or phone to the robot's Wi-Fi and open
 * http://192.168.43.1:PORT in a browser (the Control Hub's default address), or connect by USB
 * and run "adb forward tcp:PORT tcp:PORT" and open http://localhost:PORT.
 */
public class VelocityGraph {
    private static final String TAG = "VelocityGraph";
    public static final int PORT = 8000;

    private static final String PAGE = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            + "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            + "<title>Shooter Velocity</title>"
            + "<style>body{font-family:sans-serif;margin:8px;background:#fff;color:#222}"
            + "#v,#res{font-size:32px;font-weight:bold}button{font-size:18px;padding:4px 12px}"
            + "canvas{width:100%;height:65vh;border:1px solid #ccc;cursor:crosshair}"
            + "table{border-collapse:collapse;margin-top:6px}"
            + "td,th{border:1px solid #ccc;padding:4px 10px;text-align:right}th{background:#f0f0f0}"
            + "tr.best td{background:#c8e6c9}td.top{font-weight:bold;color:#1b5e20}"
            + "</style></head><body>"
            + "<div>Shooter velocity (ticks/sec): <span id='v'>--</span> "
            + "<button id='f'>Freeze</button> <button id='clr'>Clear marks</button> "
            + "<button id='rst'>Reset shots</button> <button id='mm'>Min/Max: off</button> "
            + "<button id='an'>analyze me</button> "
            + "<button id='sv'>save this data to a file</button></div>"
            + "<div>Spin-up time: <span id='res'>--</span></div>"
            + "<div>Shot recovery: <span id='shot'>--</span></div>"
            + "<canvas id='c'></canvas>"
            + "<div>Hover over the line to read time and speed. Click to keep a mark.</div>"
            + "<div id='out' style='display:none'></div>"
            + "<script>"
            + "var WINDOW_MS=15000,KEEP_MS=60000,pts=[],saved=[],frozen=false,frozenAt=0,idle=true,showMM=false,"
            + "startT=null,reachT=null,target=null,pval=null,fval=null,mouseX=null,W=0,H=0,END=0,"
            + "shots=[],inDip=false,dipStart=0,dipBase=0,minV=0,base=null,DIP_FRAC=0.04,"
            + "BAND_MIN_FRAC=0.01,SETTLE_MS=500,inBandSince=null,stableDev=0,baseDev=0,dipBand=0,"
            + "hist=[],excl=[],shotLog=[],cfgList=[],cfgMap={},curKey=null,curCi=-1,inTrans=false,"
            + "lastList=[],lastBest=null,"
            + "transStart=0,dipCi=0,"
            + "shotEl=document.getElementById('shot'),"
            + "c=document.getElementById('c'),g=c.getContext('2d'),"
            + "res=document.getElementById('res');"
            // Mean of the last second if the velocity has settled, otherwise null. Also records
            // the largest deviation from that mean (the normal noise) in stableDev.
            + "function stable(now){var r=pts.filter(function(p){return now-p.t<=1000});"
            + "if(r.length<10)return null;var s=0;r.forEach(function(p){s+=p.v});var m=s/r.length;"
            + "if(m<=50)return null;var d=0;"
            + "for(var i=0;i<r.length;i++){var e=Math.abs(r[i].v-m);if(e>0.03*m)return null;"
            + "d=Math.max(d,e)}stableDev=d;return m}"
            // Spin-up starts when velocity rises above 50 and ends at 95% of the settled value.
            + "function spinUp(t,v){"
            + "if(v<=50){idle=true;if(reachT===null){if(startT!==null)excl.push({start:startT,end:t});"
            + "startT=null;res.textContent='--'}return}"
            + "if(idle){idle=false;startT=t;reachT=null;res.textContent='Measuring...'}"
            + "if(reachT!==null)return;var m=stable(t);if(m===null)return;"
            + "for(var i=0;i<pts.length;i++){"
            + "if(pts[i].t>=startT&&pts[i].v>=0.95*m){reachT=pts[i].t;break}}"
            + "if(reachT!==null){excl.push({start:startT,end:reachT});"
            + "res.textContent=((reachT-startT)/1000).toFixed(2)"
            + "+' s to reach 95% of '+Math.round(m)}}"
            // A sudden dip below the last steady speed is a shot. Recovery is timed from the
            // start of the dip until the speed is back within its normal pre-shot noise
            // (at least 1%) of that steady speed and stays there for 0.5 s.
            + "function showShots(){var n=shots.length;if(!n){shotEl.textContent='--';return}"
            + "var l=shots[n-1],s=0;shots.forEach(function(x){s+=x.secs});"
            + "shotEl.textContent=l.secs.toFixed(2)+' s (dip '+Math.round(l.base)+' to '"
            + "+Math.round(l.min)+', within +/-'+Math.round(l.band)+'), avg '+(s/n).toFixed(2)"
            + "+' s over '+n+' shot'+(n>1?'s':'')}"
            + "function shotTrack(t,v){"
            + "if(v<=50){if(inDip)excl.push({start:dipStart,end:t});inDip=false;base=null;return}"
            + "if(!inDip){var m=stable(t);if(m!==null){base=m;baseDev=stableDev;return}"
            + "if(base!==null&&v<base*(1-DIP_FRAC)){inDip=true;dipBase=base;minV=v;dipStart=t;"
            + "dipBand=Math.max(1.2*baseDev,BAND_MIN_FRAC*base);inBandSince=null;dipCi=curCi;"
            + "for(var i=pts.length-1;i>=0;i--){if(pts[i].v>=base*0.99){dipStart=pts[i].t;break}}"
            + "shotEl.textContent='Recovering...'}return}"
            + "minV=Math.min(minV,v);"
            + "if(Math.abs(v-dipBase)<=dipBand){if(inBandSince===null)inBandSince=t;"
            + "if(t-inBandSince>=SETTLE_MS){inDip=false;var sh={start:dipStart,end:inBandSince,"
            + "base:dipBase,min:minV,band:dipBand,c:dipCi,secs:(inBandSince-dipStart)/1000};"
            + "shots.push(sh);shotLog.push(sh);excl.push({start:dipStart,end:inBandSince});"
            + "showShots();return}}"
            + "else inBandSince=null;"
            + "if(t-dipStart>10000){inDip=false;base=null;excl.push({start:dipStart,end:t});"
            + "shotEl.textContent='No recovery'}}"
            // After a change of target speed or P, samples are not steady until the speed settles.
            + "function transTrack(t){if(!inTrans||t-transStart<1000)return;"
            + "if(stable(t)===null)return;inTrans=false;excl.push({start:transStart,end:t})}"
            + "function track(t,v){spinUp(t,v);shotTrack(t,v);transTrack(t)}"
            + "function X(t){return W-(END-t)/WINDOW_MS*W}"
            + "function nearest(mx){var best=null,bd=1e9;pts.forEach(function(p){"
            + "if(END-p.t>WINDOW_MS||p.t>END)return;var d=Math.abs(X(p.t)-mx);"
            + "if(d<bd){bd=d;best=p}});return best}"
            // Time is shown after the spin-up start when there is one, otherwise relative to now.
            + "function label(p){if(startT!==null)return((p.t-startT)/1000).toFixed(2)+' s after start';"
            + "return((p.t-END)/1000).toFixed(2)+' s (vs now)'}"
            + "function tip(text,x,y){g.font='14px sans-serif';var tw=g.measureText(text).width+10,"
            + "bx=Math.min(Math.max(2,x-tw/2),W-tw-2),by=Math.max(2,y-30);"
            + "g.fillStyle='rgba(255,255,255,0.95)';g.fillRect(bx,by,tw,20);"
            + "g.strokeStyle='#999';g.lineWidth=1;g.strokeRect(bx,by,tw,20);"
            + "g.fillStyle='#222';g.fillText(text,bx+5,by+15)}"
            + "function dot(x,y,col){g.fillStyle=col;g.beginPath();g.arc(x,y,5,0,6.2832);g.fill()}"
            + "function draw(){var dpr=window.devicePixelRatio||1;"
            + "c.width=Math.round(c.clientWidth*dpr);c.height=Math.round(c.clientHeight*dpr);"
            + "g.setTransform(dpr,0,0,dpr,0,0);"
            + "W=c.clientWidth;H=c.clientHeight-22;END=frozen?frozenAt:Date.now();"
            + "var max=100;if(target!==null)max=Math.max(max,target);"
            + "pts.forEach(function(p){if(END-p.t<=WINDOW_MS)max=Math.max(max,p.v)});"
            + "max=Math.ceil(max*1.1/100)*100;"
            + "function Y(v){return H-v/max*H}"
            + "g.clearRect(0,0,W,c.clientHeight);g.font='14px sans-serif';g.lineWidth=1;"
            + "for(var i=0;i<=4;i++){var y=H-i*H/4;g.strokeStyle='#ddd';g.beginPath();"
            + "g.moveTo(0,y);g.lineTo(W,y);g.stroke();g.fillStyle='#666';"
            + "g.fillText(Math.round(max*i/4),4,y-3)}"
            + "for(var s=0;s<=WINDOW_MS/1000;s++){var x=W-s*1000/WINDOW_MS*W;"
            + "g.strokeStyle='#eee';g.beginPath();g.moveTo(x,0);g.lineTo(x,H);g.stroke();"
            + "g.fillStyle='#666';"
            + "g.fillText(s===0?'now':'-'+s+'s',Math.max(2,Math.min(x-10,W-30)),H+16)}"
            + "if(target!==null){var ty=Y(target);g.strokeStyle='#ef6c00';g.lineWidth=2;"
            + "g.setLineDash([10,5]);g.beginPath();g.moveTo(0,ty);g.lineTo(W,ty);g.stroke();"
            + "g.setLineDash([]);g.fillStyle='#ef6c00';g.fillText('target '+Math.round(target),44,ty-5)}"
            + "function mark(t,text,col){if(t===null||END-t>WINDOW_MS)return;var x=X(t);"
            + "g.strokeStyle=col;g.lineWidth=2;g.setLineDash([6,4]);g.beginPath();"
            + "g.moveTo(x,0);g.lineTo(x,H);g.stroke();g.setLineDash([]);"
            + "g.fillStyle=col;g.fillText(text,x+4,14)}"
            + "mark(startT,'start','#2e7d32');mark(reachT,'at speed','#c62828');"
            // Min and max of the visible samples that are not spin-up or part of a shot.
            + "if(showMM){var lo=null,hi=null;pts.forEach(function(p){"
            + "if(p.v<=50||END-p.t>WINDOW_MS||p.t>END)return;"
            + "if(inDip&&p.t>=dipStart)return;"
            + "if(startT!==null&&p.t>=startT&&(reachT===null||p.t<=reachT))return;"
            + "for(var i=0;i<shots.length;i++){if(p.t>=shots[i].start&&p.t<=shots[i].end)return}"
            + "if(lo===null||p.v<lo)lo=p.v;if(hi===null||p.v>hi)hi=p.v});"
            + "if(lo!==null){[[hi,'max ',-4],[lo,'min ',14]].forEach(function(a){var y=Y(a[0]);"
            + "g.strokeStyle='#00897b';g.lineWidth=1.5;g.setLineDash([3,3]);g.beginPath();"
            + "g.moveTo(0,y);g.lineTo(W,y);g.stroke();g.setLineDash([]);g.fillStyle='#00897b';"
            + "g.fillText(a[1]+Math.round(a[0])+(a[2]<0?' (range '+Math.round(hi-lo)+')':''),"
            + "W-170,y+a[2])})}}"
            + "shots.concat(inDip?[{start:dipStart,end:END,secs:null}]:[]).forEach(function(s){"
            + "if(END-s.end>WINDOW_MS)return;var x1=Math.max(0,X(s.start)),x2=Math.min(W,X(s.end));"
            + "g.fillStyle='rgba(106,27,154,0.15)';g.fillRect(x1,0,Math.max(2,x2-x1),H);"
            + "g.fillStyle='#6a1b9a';g.fillText(s.secs!==null?s.secs.toFixed(2)+' s':'recovering...',"
            + "x1+4,34)});"
            + "g.strokeStyle='#1565c0';g.lineWidth=2;g.beginPath();var first=true;"
            + "pts.forEach(function(p){if(END-p.t>WINDOW_MS||p.t>END)return;"
            + "var x=X(p.t),y=Y(p.v);if(first){g.moveTo(x,y);first=false}else g.lineTo(x,y)});"
            + "g.stroke();"
            + "saved.forEach(function(s){if(END-s.t>WINDOW_MS||s.t>END)return;"
            + "var x=X(s.t),y=Y(s.v);dot(x,y,'#c62828');"
            + "tip(Math.round(s.v)+' ticks/s, '+s.label,x,y)});"
            + "if(mouseX!==null){var p=nearest(mouseX);if(p){var hx=X(p.t),hy=Y(p.v);"
            + "g.strokeStyle='#999';g.lineWidth=1;g.beginPath();g.moveTo(hx,0);g.lineTo(hx,H);g.stroke();"
            + "dot(hx,hy,'#1565c0');tip(Math.round(p.v)+' ticks/s, '+label(p),hx,hy)}}"
            + "g.font='bold 18px sans-serif';g.fillStyle='#222';"
            + "var pt='P = '+(pval!==null?fmt(pval):'--')+'   F = '+(fval!==null?fmt(fval):'--');"
            + "g.fillText(pt,W-g.measureText(pt).width-8,20);g.font='14px sans-serif'}"
            + "function num(s){var n=parseFloat(s);return isNaN(n)?null:n}"
            + "function fmt(x){return x===null?'--':Math.round(x*10)/10}"
            // Summary per (target speed, P): steady min/max, shots and average recovery time.
            // Spin-ups, shots and the settling after a change of speed or P are left out of min/max.
            + "function analyze(){var rows={},now=Date.now(),open=excl.slice();"
            + "if(startT!==null&&reachT===null)open.push({start:startT,end:now});"
            + "if(inDip)open.push({start:dipStart,end:now});"
            + "if(inTrans)open.push({start:transStart,end:now});"
            + "function row(c){if(!rows[c])rows[c]={c:cfgList[c],min:null,max:null,n:0,s:0,q:0,secs:[]};"
            + "return rows[c]}"
            + "hist.forEach(function(h){if(h.v<=50)return;"
            + "for(var i=0;i<open.length;i++){if(h.t>=open[i].start&&h.t<=open[i].end)return}"
            + "var r=row(h.c);r.n++;r.s+=h.v;r.q+=h.v*h.v;if(r.min===null||h.v<r.min)r.min=h.v;"
            + "if(r.max===null||h.v>r.max)r.max=h.v});"
            + "shotLog.forEach(function(s){row(s.c).secs.push(s.secs)});"
            + "var list=[];for(var k in rows){var r=rows[k];if(r.n>=10||r.secs.length)list.push(r)}"
            + "list.sort(function(a,b){return(a.c.target-b.c.target)||(a.c.p-b.c.p)||(a.c.f-b.c.f)});"
            // Variance is the standard deviation of the steady speed. The best setting has the
            // fastest average recovery; std dev only decides between recoveries within 0.05 s.
            + "list.forEach(function(r){"
            + "r.sd=r.n?Math.sqrt(Math.max(0,r.q/r.n-Math.pow(r.s/r.n,2))):null;"
            + "var s=0;r.secs.forEach(function(x){s+=x});r.avg=r.secs.length?s/r.secs.length:null});"
            + "var ok=list.filter(function(r){return r.sd!==null&&r.avg!==null}),best=null,"
            + "lowSd=null,fastRec=null;"
            + "if(ok.length>=2){var bySd=ok.slice().sort(function(a,b){return a.sd-b.sd}),"
            + "byAvg=ok.slice().sort(function(a,b){return a.avg-b.avg});lowSd=bySd[0];fastRec=byAvg[0];"
            + "best=fastRec;ok.forEach(function(r){"
            + "if(r.avg-fastRec.avg<=0.05&&r.sd<best.sd)best=r})}"
            + "function td(x,top){return'<td'+(top?' class=\"top\"':'')+'>'+x+'</td>'}"
            + "lastList=list;lastBest=best;"
            + "var out=document.getElementById('out'),h;"
            + "if(!list.length){h='<p>Not enough data yet. Run the shooter at a steady speed.</p>'}"
            + "else{h='<table><tr><th>Target velocity</th><th>P</th><th>F</th><th>Min</th><th>Max</th>"
            + "<th>Range</th><th>Std dev</th><th>Shots</th><th>Avg recovery</th></tr>';"
            + "list.forEach(function(r){"
            + "h+=(r===best?'<tr class=\"best\">':'<tr>')+td(fmt(r.c.target))+td(fmt(r.c.p))"
            + "+td(fmt(r.c.f))"
            + "+td(r.n?Math.round(r.min):'--')+td(r.n?Math.round(r.max):'--')"
            + "+td(r.n?Math.round(r.max-r.min):'--')+td(r.sd===null?'--':r.sd.toFixed(1),r===lowSd)"
            + "+td(r.secs.length)+td(r.avg===null?'--':r.avg.toFixed(2)+' s',r===fastRec)+'</tr>'});"
            + "h+='</table>';"
            + "if(best)h+='<p><b>Best: target '+fmt(best.c.target)+', P '+fmt(best.c.p)+', F '"
            + "+fmt(best.c.f)+'</b> (std dev '"
            + "+best.sd.toFixed(1)+', avg recovery '+best.avg.toFixed(2)+' s). Green row: fastest "
            + "average recovery (std dev only breaks ties within 0.05 s). Bold: lowest variance, "
            + "fastest recovery.</p>';"
            + "else h+='<p>The best setting is highlighted once two settings each have steady running "
            + "and at least one shot.</p>';"
            + "h+='<p>Min, Max and Std dev are steady speeds between shots (spin-up, shots and speed "
            + "changes excluded). Avg recovery is from the start of a dip until the speed is "
            + "back to normal.</p>'}"
            + "out.innerHTML=h+'<p><button id=\"hide\">Hide</button></p>';"
            + "document.getElementById('hide').onclick=function(){out.style.display='none'};"
            + "out.style.display='block'}"
            + "function poll(){if(frozen)return;var x=new XMLHttpRequest();x.open('GET','/v');"
            + "x.onload=function(){var a=x.responseText.split(',');var v=num(a[0]);"
            + "if(v===null||frozen)return;target=num(a[1]);pval=num(a[2]);fval=num(a[3]);"
            + "var t=Date.now();"
            + "var key=(target===null?'-':Math.round(target))+'|'+(pval===null?'-':pval)"
            + "+'|'+(fval===null?'-':fval);"
            + "if(key!==curKey){curKey=key;"
            + "if(!(key in cfgMap)){cfgMap[key]=cfgList.length;"
            + "cfgList.push({target:target,p:pval,f:fval})}"
            + "curCi=cfgMap[key];if(!inTrans){inTrans=true;transStart=t}}"
            + "hist.push({t:t,v:v,c:curCi});if(hist.length>200000)hist.splice(0,20000);"
            + "pts.push({t:t,v:v});"
            + "pts=pts.filter(function(p){return t-p.t<=KEEP_MS});track(t,v);"
            + "document.getElementById('v').textContent=Math.round(v);draw()};x.send()}"
            + "document.getElementById('f').onclick=function(){frozen=!frozen;"
            + "if(frozen)frozenAt=Date.now();this.textContent=frozen?'Resume':'Freeze';draw()};"
            + "document.getElementById('mm').onclick=function(){showMM=!showMM;"
            + "this.textContent=showMM?'Min/Max: on':'Min/Max: off';draw()};"
            + "document.getElementById('an').onclick=analyze;"
            // Saves the same table that analyze shows, as a CSV file download.
            + "document.getElementById('sv').onclick=function(){analyze();if(!lastList.length)return;"
            + "var rows=[['target_velocity','p','f','min','max','range','std_dev','shots',"
            + "'avg_recovery_s','best']];"
            + "lastList.forEach(function(r){var n=function(x){return x===null?'':x};"
            + "rows.push([n(r.c.target),n(r.c.p),n(r.c.f),r.n?Math.round(r.min):'',"
            + "r.n?Math.round(r.max):'',r.n?Math.round(r.max-r.min):'',"
            + "r.sd===null?'':r.sd.toFixed(2),r.secs.length,r.avg===null?'':r.avg.toFixed(3),"
            + "r===lastBest?'yes':''])});"
            + "var csv=rows.map(function(r){return r.join(',')}).join('\\n')+'\\n';"
            + "var a=document.createElement('a');"
            + "a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));"
            + "a.download='velocity-data-'+new Date().toISOString().slice(0,19).replace(/[:T]/g,'-')"
            + "+'.csv';document.body.appendChild(a);a.click();document.body.removeChild(a)};"
            + "document.getElementById('clr').onclick=function(){saved=[];draw()};"
            + "document.getElementById('rst').onclick=function(){shots=[];inDip=false;"
            + "shotEl.textContent='--';draw()};"
            + "c.onmousemove=function(e){mouseX=e.offsetX;if(frozen)draw()};"
            + "c.onmouseleave=function(){mouseX=null;if(frozen)draw()};"
            + "c.onclick=function(e){var p=nearest(e.offsetX);"
            + "if(p){saved.push({t:p.t,v:p.v,label:label(p)});if(frozen)draw()}};"
            + "setInterval(poll,50);"
            + "</script></body></html>";

    private volatile double value;
    private volatile double target = Double.NaN;
    private volatile double pValue = Double.NaN;
    private volatile double fValue = Double.NaN;
    private volatile boolean running;
    private ServerSocket serverSocket;
    private Thread thread;

    public void start() {
        try {
            serverSocket = new ServerSocket();
            serverSocket.setReuseAddress(true);
            serverSocket.bind(new InetSocketAddress(PORT));
        } catch (IOException e) {
            Log.e(TAG, "Could not start graph server on port " + PORT, e);
            return;
        }
        running = true;
        thread = new Thread(this::serve, TAG);
        thread.setDaemon(true);
        thread.start();
    }

    public void setVelocity(double newValue) {
        value = newValue;
    }

    public void setTarget(double newTarget) {
        target = newTarget;
    }

    public void setPValue(double newPValue) {
        pValue = newPValue;
    }

    public void setFValue(double newFValue) {
        fValue = newFValue;
    }

    public void stop() {
        running = false;
        try {
            if (serverSocket != null) {
                serverSocket.close();
            }
        } catch (IOException e) {
            Log.w(TAG, "Error closing graph server", e);
        }
    }

    private void serve() {
        while (running) {
            try (Socket socket = serverSocket.accept()) {
                socket.setSoTimeout(1000);
                BufferedReader in = new BufferedReader(
                        new InputStreamReader(socket.getInputStream(), StandardCharsets.US_ASCII));
                String requestLine = in.readLine();
                boolean wantsValue = requestLine != null && requestLine.startsWith("GET /v");
                String body = wantsValue
                        ? value + "," + target + "," + pValue + "," + fValue : PAGE;
                String type = wantsValue ? "text/plain" : "text/html";
                byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
                String header = "HTTP/1.1 200 OK\r\nContent-Type: " + type
                        + "; charset=utf-8\r\nContent-Length: " + bytes.length
                        + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
                OutputStream out = socket.getOutputStream();
                out.write(header.getBytes(StandardCharsets.US_ASCII));
                out.write(bytes);
                out.flush();
            } catch (IOException e) {
                if (running) {
                    Log.w(TAG, "Graph request failed", e);
                }
            }
        }
    }
}
