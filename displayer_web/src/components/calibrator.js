import React, {Component, createRef} from 'react';
import * as zrender from 'zrender'
import { connect } from 'umi'


@connect(({ configProvider }) => ({
  configProvider
}))
class Calibrator extends Component {
  instance = createRef();
  container = createRef();
  name = this.props.name;
  zr = null;
  rate = this.props.configProvider.calibrator.rate;
  clipRectRate = this.props.configProvider.calibrator.clipRectRate;
  path = this.props.configProvider.calibrator.cameras[this.name].path;
  aspectRatio = this.props.configProvider.calibrator.cameras[this.name].aspectRatio;
  imgUrl = '/api/camera?cam=' + encodeURIComponent(this.name);
  w = 0
  h = 0
  state = {width: '90%', height: '10%'}

  componentDidMount() {
    let isMouseDown = false;
    this.zr = zrender.init(this.instance.current);
    this.w = this.zr.getWidth();
    this.h  = this.zr.getHeight();
    const that = this
    const convert2Screen = p => [p[0] * that.w, p[1] * that.h]

    this.zr.on('mousedown', (e) => {
      isMouseDown = e.target?.id;
      if ((isMouseDown || isMouseDown === 0) && isMouseDown < this.path.length && isMouseDown >= 0) {
        clipRect.show()
        img.show()
        line1.show()
        line2.show()
        fillRect.show()
      }
    });
    this.zr.on('mouseup', (e) => {
      if ((isMouseDown || isMouseDown === 0) && isMouseDown < this.path.length && isMouseDown >= 0) {
        mouseMove(circle[isMouseDown])
        clipRect.hide()
        img.hide()
        line1.hide()
        line2.hide()
        fillRect.hide()
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
          points: this.path.map(convert2Screen),
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
          points: [...this.path, this.path[0]].map(convert2Screen),
        }
      }
    )
    let circle = [];
    for (let i = 0 ; i < this.path.length; i++) {
      circle.push(new zrender.Circle({
        id: i,
        style: {
          fill: '#fff',
        },
        shape: {
          cx: 0,
          cy: 0,
          r: 5,
        },
        position: convert2Screen(this.path[i]),
        draggable: true,
      }).on('mousemove', (e) => {
        if (isMouseDown || isMouseDown === 0) {
          mouseMove(e.target)
        }
        let rect = getBox(e.target);
        console.log(rect)
        clipRect.setShape({
          x: rect.x,
          y: rect.y,
          width: rect.width,
          height: rect.height,
        })
        fillRect.setShape({
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
        width: this.w * this.clipRectRate,
        height: this.h * this.clipRectRate,
      },
      zlevel: 3
    })
    clipRect.hide()

    let img = new zrender.Image({
      style: {
        image: this.imgUrl,
        x: 0,
        y: 0,
        width: this.w * this.rate,
        height: this.h * this.rate,
      },
      zlevel: 2
    })
    img.setClipPath(clipRect)
    img.hide()

    let fillRect = new zrender.Rect({
      style: {
        fill: '#000',
      },
      shape: {
        x: 0,
        y: 0,
        width: this.w * this.clipRectRate,
        height: this.h * this.clipRectRate,
      },
      zlevel: 1
    })
    fillRect.hide()

    let line1 = new zrender.Line({
      style: {
        stroke: '#fff',
        lineWidth: 2,
      },
      shape: {
        x1: 0,
        y1: this.h * this.clipRectRate / 2,
        x2: this.w,
        y2: this.h * this.clipRectRate / 2,
      },
      zlevel: 3
    })
    let line2 = new zrender.Line({
      style: {
        stroke: '#fff',
        lineWidth: 2,
      },
      shape: {
        x1: this.w * this.clipRectRate / 2,
        y1: 0,
        x2: this.w * this.clipRectRate / 2,
        y2: this.h,
      },
      zlevel: 3
    })
    line1.hide()
    line2.hide()

    this.zr.add(clipRect);
    this.zr.add(fillRect);
    this.zr.add(img);
    this.zr.add(line1);
    this.zr.add(line2);

    function mouseMove(e) {
      that.path[e.id] = [e.x / that.w, e.y / that.h];
      polygon.setShape({
        points: that.path.map(convert2Screen),
      });
      polyline.setShape({
        points: [...that.path, that.path[0]].map(convert2Screen),
      });
    }

    function getBox(e) {
      let width = that.w * that.clipRectRate;
      let height = that.h * that.clipRectRate;
      let x = 0, y = 0;
      if (e.x < that.w / 2) {
        x = that.w - width;
      }
      if (e.y < that.h / 2) {
        y = that.h - height;
      }
      const imgX = x + width / 2 - e.x * that.rate;
      const imgY = y + height / 2 - e.y * that.rate;
      return {
        x, y, width, height, imgX, imgY,
      }
    }

    function resize() {
      let targetWidth = that.container.current.parentNode.clientWidth - 10;
      that.setState({
        width: targetWidth.toString() + 'px',
        height: (Math.round(targetWidth / that.aspectRatio)).toString() + 'px'
      })
      that.zr.resize();
      that.w = that.zr.getWidth();
      that.h = that.zr.getHeight();
      polygon.setShape({
        points: that.path.map(convert2Screen),
      });
      polyline.setShape({
        points: [...that.path, that.path[0]].map(convert2Screen),
      });
      for(let i = 0; i < circle.length; i++) {
        circle[i].attr('position', convert2Screen(that.path[i]));
      }
      img.attr({
        style: {
          width: that.w * that.rate,
          height: that.h * that.rate,
        }
      })
    }
    setTimeout(() => {resize()}, 100)
    window.addEventListener('resize', resize);
  }

  render() {
    return (
      <div>
        <div style={{ position: 'relative', width: this.state.width, height: this.state.height, padding: '10px' }} ref={ this.container }>
          <div style={{ position: 'absolute', top: '5px', left: '5px', width: '100%', height: '100%', zIndex: 3 }}>
            <div ref={this.instance} style={{width: '100%', height: '100%'}} />
          </div>
          <div style={{ position: 'absolute', top: '5px', left: '5px', width: '100%', height: '100%', zIndex: 2 }}>
            <img style={{width: '100%', height: '100%'}} src={ this.imgUrl } alt="cameraOne"/>
          </div>
        </div>
      </div>
    );
  }
}

export default Calibrator;
