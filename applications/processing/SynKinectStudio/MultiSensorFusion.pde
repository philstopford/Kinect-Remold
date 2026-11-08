// Multi-sensor body fusion primitive. Hosts send calibrated BodyFrame observations
// to a coordinator; the coordinator uses this resolver before MocapRetargeter.
final class MultiSensorBodyObservation {
  final BodyFrame frame;final String hostId,calibrationRevision;final long timestampNs,clockUncertaintyNs;
  final float[] r,t; // camera -> room rigid transform, row-major 3x3 + metres
  MultiSensorBodyObservation(BodyFrame frame,String hostId,String revision,long timestampNs,long uncertaintyNs,float[] r,float[] t){
    if(frame==null||r==null||r.length!=9||t==null||t.length!=3)throw new IllegalArgumentException("observation");
    this.frame=frame;this.hostId=hostId==null?"":hostId;calibrationRevision=revision==null?"":revision;this.timestampNs=timestampNs;clockUncertaintyNs=uncertaintyNs;this.r=r.clone();this.t=t.clone();
  }
  PVector room(int joint){float x=frame.x(joint),y=frame.y(joint),z=frame.z(joint);return new PVector(r[0]*x+r[1]*y+r[2]*z+t[0],r[3]*x+r[4]*y+r[5]*z+t[1],r[6]*x+r[7]*y+r[8]*z+t[2]);}
}
class MultiSensorBodyFusion {
  // Selects the best contemporaneous unobscured joint observation. A candidate
  // far from the confidence-weighted consensus is rejected, never averaged in.
  BodyFrame fuse(ArrayList<MultiSensorBodyObservation> views,long targetNs,long windowNs,String revision){
    if(views==null||views.isEmpty())return null;MultiSensorBodyObservation base=views.get(0);int n=base.frame.jointCount();float[] xyz=new float[n*3],conf=new float[n];int[] state=new int[n];
    for(int j=0;j<n;j++){ArrayList<MultiSensorBodyObservation> valid=new ArrayList<MultiSensorBodyObservation>();for(MultiSensorBodyObservation v:views)
      if(v.calibrationRevision.equals(revision)&&Math.abs(v.timestampNs-targetNs)<=windowNs+v.clockUncertaintyNs&&v.frame.jointTracked(j))valid.add(v);
      if(valid.isEmpty())continue;PVector c=new PVector();float w=0;for(MultiSensorBodyObservation v:valid){float q=v.frame.confidence(j);PVector p=v.room(j);c.add(PVector.mult(p,q));w+=q;}c.div(max(.0001f,w));
      MultiSensorBodyObservation best=null;float score=-1;for(MultiSensorBodyObservation v:valid){float d=PVector.dist(v.room(j),c);float s=v.frame.confidence(j)/(1f+d/.18f);if(s>score){score=s;best=v;}}
      PVector p=best.room(j);xyz[j*3]=p.x;xyz[j*3+1]=p.y;xyz[j*3+2]=p.z;conf[j]=best.frame.confidence(j);state[j]=best.frame.state(j);
    }
    return new BodyFrame("fusion:"+revision,targetNs/1000000L,targetNs/1000000L,base.frame.frameNumber,base.frame.trackingId,true,1,xyz,conf,state,new float[]{0,-1,0},false,0);
  }
}
