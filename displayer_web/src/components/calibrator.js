import React, {Component, createRef} from 'react';
import * as zrender from 'zrender'
import { connect } from 'umi'


@connect(({ configProvider }) => ({
  configProvider
}))
class Calibrator extends Component {
  instance = createRef();
  name = this.props.name;
  zr = null;
  rate = this.props.configProvider.calibrator.rate;
  clipRectRate = this.props.configProvider.calibrator.clipRectRate;
  path = this.props.configProvider.calibrator.cameras[this.name].path;
  imgUrl = '/api/camera?cam=' + encodeURIComponent(this.name);

  componentDidMount() {
    console.log(this.imgUrl)
    let isMouseDown = false;
    this.zr = zrender.init(this.instance.current);
    let w = this.zr.getWidth();
    let h = this.zr.getHeight();
    this.zr.on('mousedown', (e) => {
      isMouseDown = e.target?.id;
      if ((isMouseDown || isMouseDown === 0) && isMouseDown < this.path.length && isMouseDown >= 0) {
        clipRect.show()
        img.show()
        line1.show()
        line2.show()
      }
    });
    this.zr.on('mouseup', (e) => {
      if ((isMouseDown || isMouseDown === 0) && isMouseDown < this.path.length && isMouseDown >= 0) {
        mouseMove(circle[isMouseDown])
        clipRect.hide()
        img.hide()
        line1.hide()
        line2.hide()
      }
      isMouseDown = false;
    });
    let polygon = new zrender.Polygon(
      {
        style: {
          fill: '#fff',
          opacity: 0.3,
        },
        shape: {
          points: this.path,
        }
      }
    )
    let polyline = new zrender.Polyline(
      {
        style: {
          stroke: '#fff',
          lineWidth: 2,
        },
        shape: {
          points: [...this.path, this.path[0]],
        }
      }
    )
    let circle = [];
    for (let i = 0 ; i < this.path.length; i++) {
      circle.push(new zrender.Circle(
        {
          id: i,
          style: {
            fill: '#fff',
          },
          shape: {
            cx: 0,
            cy: 0,
            r: 5,
          },
          position: this.path[i],
          draggable: true,
        }
      ).on('mousemove', (e) => {
        if (isMouseDown || isMouseDown === 0) {
          mouseMove(e.target)
        }
        let rect = getBox(e.target);
        clipRect.setShape({
          x: rect.x,
          y: rect.y,
          width: rect.width,
          height: rect.height,
        })
        line1.setShape({
          x1: rect.x,
          y1: rect.y + rect.height / 2,
          x2: rect.x + rect.width,
          y2: rect.y + rect.height / 2,
        })
        line2.setShape({
          x1: rect.x + rect.width / 2,
          y1: rect.y,
          x2: rect.x + rect.width / 2,
          y2: rect.y + rect.height,
        })
        img.attr({
          style: {
            x: rect.imgX,
            y: rect.imgY,
          }
        })
      }));
    }
    this.zr.add(polygon);
    this.zr.add(polyline);
    for (let i = 0; i < circle.length; i++) {
      this.zr.add(circle[i]);
    }

    let clipRect = new zrender.Rect({
      style: {
        fill: 'none',
        stroke: '#fff',
        lineWidth: 2,
      },
      shape: {
        x: 0,
        y: 0,
        width: w * this.clipRectRate,
        height: h * this.clipRectRate,
      },
      zlevel: 2
    })
    clipRect.hide()

    let img = new zrender.Image({
      style: {
        image: this.imgUrl,
        x: 0,
        y: 0,
        width: w * this.rate,
        height: h * this.rate,
      },
      zlevel: 1
    })
    img.setClipPath(clipRect)
    img.hide()

    let line1 = new zrender.Line({
      style: {
        stroke: '#fff',
        lineWidth: 2,
      },
      shape: {
        x1: 0,
        y1: h * this.clipRectRate / 2,
        x2: w,
        y2: h * this.clipRectRate / 2,
      },
      zlevel: 2
    })
    let line2 = new zrender.Line({
      style: {
        stroke: '#fff',
        lineWidth: 2,
      },
      shape: {
        x1: w * this.clipRectRate / 2,
        y1: 0,
        x2: w * this.clipRectRate / 2,
        y2: h,
      },
      zlevel: 2
    })
    line1.hide()
    line2.hide()

    this.zr.add(clipRect);
    this.zr.add(img);
    this.zr.add(line1);
    this.zr.add(line2);

    const that = this;
    function mouseMove(e) {
      that.path[e.id] = [e.x, e.y];
      polygon.setShape({
        points: that.path,
      });
      polyline.setShape({
        points: [...that.path, that.path[0]],
      });
    }

    function getBox(e) {
      let width = w * that.clipRectRate;
      let height = h * that.clipRectRate;
      let x = 0, y = 0;
      if (e.x < w / 2) {
        x = w - width;
      }
      if (e.y < h / 2) {
        y = h - height;
      }
      const imgX = x + width / 2 - e.x * that.rate;
      const imgY = y + height / 2 - e.y * that.rate;
      return {
        x, y, width, height, imgX, imgY,
      }
    }
  }

  render() {
    return (
      <div>
        <div style={{ position: 'relative', width: '640px', height: '480px' }}>
          <div style={{ position: 'absolute', top: '0px', left: '0px', width: '100%', height: '100%', zIndex: 3 }}>
            <div ref={this.instance} style={{width: '100%', height: '100%'}} />
          </div>
          <div style={{ position: 'absolute', top: '0px', left: '0px', width: '100%', height: '100%', zIndex: 2 }}>
            <img style={{width: '100%', height: '100%'}} src={ this.imgUrl } alt="cameraOne"/>
          </div>
        </div>
      </div>
    );
  }
}

export default Calibrator;
