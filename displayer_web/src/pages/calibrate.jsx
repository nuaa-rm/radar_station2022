import React, {Component, createRef} from 'react';
import { connect } from 'dva';
import Calibrator from "../components/calibrator";
import { Select, Button } from 'antd';
import { displayerBackend } from "../displayerBackend";

const { Option } = Select;

@connect(({ configProvider }) => ({
  configProvider,
}))
class Calibrate extends Component {
  state = { camera: '', height: 600 }
  container = createRef();
  calibratorDiv = createRef();
  calibrator = createRef();

  handleChange(e) {
    this.setState({
      camera: e,
    })
  }

  resize() {
    const clientHeight = this.container?.current?.clientHeight;
    if (clientHeight) {
      this.setState({
        height: clientHeight - 52,
      })
    }
  }

  componentDidMount() {
    this.resize()
    window.addEventListener('resize', ()=>{this.resize()});
  }

  save() {
    const path = this.calibrator?.current?.getPath()
    displayerBackend.saveCalibrate(this.state.camera, path)
  }

  reset() {
    this.calibrator?.current?.refresh()
  }

  render() {
    const options = []
    const cameras = Object.keys(this.props.configProvider.calibrator.cameras)
    for (let i = 0; i < cameras.length; i++) {
      if (this.props.configProvider.calibrator.cameras[cameras[i]].isCalibrated) {
        options.push(<Option key={cameras[i]}>{cameras[i]}</Option>)
      }
    }
    return (
      <div style={{ height: '100%' }} ref={this.container}>
        <div style={{ paddingBottom: 10, paddingTop: 10 }}>
          <Select style={{ width: 200 }} onChange={(e)=>this.handleChange(e)}>
            {options}
          </Select>
          <div style={{ display: 'inline', float: 'right' }}>
            <Button onClick={()=>this.reset()} disabled={!this.state.camera}>Reset</Button>&nbsp;
            <Button onClick={()=>this.save()} type="primary" disabled={!this.state.camera}>Save</Button>
          </div>
        </div>
        <div style={{ height: this.state.height }} ref={this.calibratorDiv}>
          {
            this.state.camera ?
              <Calibrator name={ this.state.camera } ref={this.calibrator} /> :
              <img src={require('../assets/noCamera.png')} alt="noCameraSelect" style={{maxWidth: '100%', maxHeight: '100%'}} />
          }
        </div>
      </div>
    );
  }
}

export default Calibrate;
