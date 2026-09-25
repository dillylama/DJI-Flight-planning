const E = require('../route-engine.js');
// synthetic ~4,000 ha irregular block near Nimba, synthetic ridge terrain 440–1,225 m
const lat0=7.55, lon0=-8.55, m2d=1/111320;
const polyM=[[-3300,-3000],[3100,-3300],[3400,800],[1500,3200],[-2800,3000],[-3500,200]];
const poly=polyM.map(([x,y])=>[lon0+x*m2d/Math.cos(lat0*Math.PI/180), lat0+y*m2d]);
const elev=(lon,lat)=>{const x=(lon-lon0)/m2d*Math.cos(lat0*Math.PI/180), y=(lat-lat0)/m2d;
  return 440 + 785*Math.exp(-((x-800)**2/1.2e6 + (y-500)**2/1.2e7)) + 60*Math.sin(x/400)*Math.cos(y/550);};
const plan=E.planLines(poly,{courseDeg:20});
const full=E.applyHeights(plan,E.buildRoute(plan),elev);
console.log('FULL',E.stats(plan,full));
const res=E.applyHeights(plan,E.buildRoute(plan,{fromLine:9,speedMs:14}),elev);
console.log('RESUME @line 9, 14 m/s',E.stats(plan,res), 'starts on line', res.startIdx);
console.log('first wps:',full.wps.slice(0,4).map(w=>[w.role,w.actions,w.lat.toFixed(6),w.lon.toFixed(6),w.h.toFixed(1),w.dampingM.toFixed(1)]));
// SVG preview
const W=760,H=1000; const all=[...plan.polyXY,...full.wps.map(w=>w.xy)];
const xs=all.map(p=>p[0]),ys=all.map(p=>p[1]);const mnx=Math.min(...xs),mxx=Math.max(...xs),mny=Math.min(...ys),mxy=Math.max(...ys);
const s=Math.min((W-40)/(mxx-mnx),(700-40)/(mxy-mny));const P=p=>[20+(p[0]-mnx)*s,20+(mxy-p[1])*s];
let svg=`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" style="background:#fff;font-family:sans-serif">`;
svg+=`<polygon points="${plan.polyXY.map(P).map(p=>p.join(',')).join(' ')}" fill="#e8f0ff" stroke="#36c"/>`;
const col={approach:'#c00',fig8:'#e80',runin:'#999',line:'#063',runout:'#999'};
for(let i=1;i<full.wps.length;i++){const a=P(full.wps[i-1].xy),b=P(full.wps[i].xy);svg+=`<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}" stroke="${col[full.wps[i].role]}" stroke-width="1.2"/>`;}
res.wps.filter(w=>w.role==='fig8').forEach(w=>{const p=P(w.xy);svg+=`<circle cx="${p[0]}" cy="${p[1]}" r="1.8" fill="#a0f"/>`});
// profile of line 6
const L=full.wps.filter(w=>w.line===6);const hs=L.flatMap(w=>[w.h,w.terrainUnderWp]);const hmn=Math.min(...hs)-50,hmx=Math.max(...hs)+50;
const px=i=>20+i*(W-40)/(L.length-1),py=h=>980-(h-hmn)/(hmx-hmn)*240;
svg+=`<text x="20" y="730" font-size="14">Line 6 profile: flight height (blue) vs terrain under waypoint (brown)</text>`;
svg+=`<polyline fill="none" stroke="#36c" stroke-width="2" points="${L.map((w,i)=>px(i)+','+py(w.h)).join(' ')}"/>`;
svg+=`<polyline fill="none" stroke="#852" stroke-width="2" points="${L.map((w,i)=>px(i)+','+py(w.terrainUnderWp)).join(' ')}"/>`;
svg+='</svg>';require('fs').writeFileSync(__dirname+'/preview.svg',svg);
