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
  w = 0
  h = 0
  state = {width: '90%', height: '10%', imgUrl: '/api/camera?cam=' + encodeURIComponent(this.name)}
  circle = []
  polygon = null
  polyline = null
  clipRect = null
  img = null
  fillRect = null
  line1 = null
  line2 = null

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
        this.clipRect.show()
        this.img.show()
        this.line1.show()
        this.line2.show()
        this.fillRect.show()
      }
    });
    this.zr.on('mouseup', () => {
      if ((isMouseDown || isMouseDown === 0) && isMouseDown < this.path.length && isMouseDown >= 0) {
        mouseMove(this.circle[isMouseDown])
        this.clipRect.hide()
        this.img.hide()
        this.line1.hide()
        this.line2.hide()
        this.fillRect.hide()
      }
      isMouseDown = false;
    });
    this.polygon = new zrender.Polygon(
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
    this.polyline = new zrender.Polyline(
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
    for (let i = 0 ; i < this.path.length; i++) {
      this.circle.push(new zrender.Circle({
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
        this.clipRect.setShape({
          x: rect.x,
          y: rect.y,
          width: rect.width,
          height: rect.height,
        })
        this.fillRect.setShape({
          x: rect.x,
          y: rect.y,
          width: rect.width,
          height: rect.height,
        })
        this.line1.setShape({
          x1: rect.x,
          y1: rect.y + rect.height / 2,
          x2: rect.x + rect.width,
          y2: rect.y + rect.height / 2,
        })
        this.line2.setShape({
          x1: rect.x + rect.width / 2,
          y1: rect.y,
          x2: rect.x + rect.width / 2,
          y2: rect.y + rect.height,
        })
        this.img.attr({
          style: {
            x: rect.imgX,
            y: rect.imgY,
          }
        })
      }));
    }
    this.zr.add(this.polygon);
    this.zr.add(this.polyline);
    for (let i = 0; i < this.circle.length; i++) {
      this.zr.add(this.circle[i]);
    }

    this.clipRect = new zrender.Rect({
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
    this.clipRect.hide()

    this.img = new zrender.Image({
      style: {
        image: this.state.imgUrl,
        x: 0,
        y: 0,
        width: this.w * this.rate,
        height: this.h * this.rate,
      },
      zlevel: 2
    })
    this.img.setClipPath(this.clipRect)
    this.img.hide()

    this.fillRect = new zrender.Rect({
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
    this.fillRect.hide()

    this.line1 = new zrender.Line({
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
    this.line2 = new zrender.Line({
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
    this.line1.hide()
    this.line2.hide()

    this.zr.add(this.clipRect);
    this.zr.add(this.fillRect);
    this.zr.add(this.img);
    this.zr.add(this.line1);
    this.zr.add(this.line2);

    function mouseMove(e) {
      that.path[e.id] = [e.x / that.w, e.y / that.h];
      that.polygon.setShape({
        points: that.path.map(convert2Screen),
      });
      that.polyline.setShape({
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
      let targetWidth = that.container?.current?.parentNode?.clientWidth;
      let clientHeight = that.container?.current?.parentNode?.clientHeight;
      if (!targetWidth || !clientHeight) {
        return ;
      }
      let targetHeight = Math.round(targetWidth / that.aspectRatio)
      if (targetHeight > clientHeight) {
        targetHeight = clientHeight;
        targetWidth = Math.round(targetHeight * that.aspectRatio);
      }
      that.setState({
        width: targetWidth.toString() + 'px',
        height: targetHeight.toString() + 'px'
      })
      that.zr.resize();
      that.w = that.zr.getWidth();
      that.h = that.zr.getHeight();
      that.polygon.setShape({
        points: that.path.map(convert2Screen),
      });
      that.polyline.setShape({
        points: [...that.path, that.path[0]].map(convert2Screen),
      });
      for(let i = 0; i < that.circle.length; i++) {
        that.circle[i].attr('position', convert2Screen(that.path[i]));
      }
      that.img.attr({
        style: {
          width: that.w * that.rate,
          height: that.h * that.rate,
        }
      })
    }
    setTimeout(() => {resize()}, 100)
    window.addEventListener('resize', resize);
  }

  refresh = (name=null) => {
    if (name) {
      this.name = name;
    } else {
      this.name = this.props.name;
    }
    this.path = this.props.configProvider.calibrator.cameras[this.name].path;
    this.aspectRatio = this.props.configProvider.calibrator.cameras[this.name].aspectRatio;
    this.setState({
      imgUrl: '/api/camera?cam=' + encodeURIComponent(this.name)
    })
    const that = this;
    const convert2Screen = p => [p[0] * that.w, p[1] * that.h];
    this.polygon.setShape({
      points: this.path.map(convert2Screen),
    });
    this.polyline.setShape({
      points: [...this.path, this.path[0]].map(convert2Screen),
    });
    for(let i = 0; i < this.circle.length; i++) {
      this.circle[i].attr('position', convert2Screen(this.path[i]));
    }
    this.img.attr({
      style: {
        image: '/api/camera?cam=' + encodeURIComponent(this.name),
      }
    })
  }

  shouldComponentUpdate(nextProps, nextState, nextContext) {
    if (this.props.name !== nextProps.name) {
      this.refresh(nextProps.name);
    }
    return true;
  }

  getPath() {
    return this.path;
  }

  reset() {
    this.path = this.props.configProvider.calibrator.cameras[this.name].path;
    this.resize()
  }

  // TODO: 使用transform实现img移动，优化渲染效率
  render() {
    return (
      <div style={{ height: '100%' }}>
        <div style={{ position: 'relative', width: this.state.width, height: this.state.height }} ref={ this.container }>
          <div style={{ position: 'absolute', top: '0px', left: '0px', width: '100%', height: '100%', zIndex: 3 }}>
            <div ref={this.instance} style={{width: '100%', height: '100%'}} />
          </div>
          <div style={{ position: 'absolute', top: '0px', left: '0px', width: '100%', height: '100%', zIndex: 2 }}>
            <img style={{width: '100%', height: '100%'}} src={ this.state.imgUrl } alt="cameraOne"/>
          </div>
        </div>
      </div>
    );
  }
}

export default Calibrator;
