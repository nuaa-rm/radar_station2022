import React, {Component, createRef} from 'react';
import { connect } from 'dva';
import Calibrator from "../components/calibrator";
import { Select } from 'antd';

const { Option } = Select;

@connect(({ configProvider }) => ({
  configProvider,
}))
class Calibrate extends Component {
  state = { camera: '', height: 600 }
  container = createRef();
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

  render() {
    // console.log(this.props);
    const options = []
    const cameras = Object.keys(this.props.configProvider.calibrator.cameras)
    for (let i = 0; i < cameras.length; i++) {
      options.push(<Option key={cameras[i]}>{cameras[i]}</Option>)
    }
    // console.log(this.state.height);
    return (
      <div style={{ height: '100%' }} ref={this.container}>
        <Select style={{ width: 200, paddingBottom: 10, paddingTop: 10 }} onChange={(e)=>this.handleChange(e)}>
          {options}
        </Select>
        <div style={{ height: this.state.height }} ref={this.calibrator}>
          {
            this.state.camera ?
              <Calibrator name={ this.state.camera } /> :
              <p>No camera selected</p>
          }
        </div>
      </div>
    );
  }
}

export default Calibrate;
