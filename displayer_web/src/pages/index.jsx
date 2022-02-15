import React, {Component} from 'react';
import { connect } from 'dva';
import { Select, Button } from 'antd';
import Calibrator from "../components/calibrator";

const { Option } = Select;

@connect(({ configProvider }) => ({
  configProvider,
}))
class Index extends Component {
  state = { camera: ''}

  handleChange(e) {
    this.setState({
      camera: e,
    })
  }

  render() {
    console.log(this.props);
    const options = []
    const cameras = Object.keys(this.props.configProvider.calibrator.cameras)
    for (let i = 0; i < cameras.length; i++) {
      options.push(<Option key={cameras[i]}>{cameras[i]}</Option>)
    }
    return (
      <div>
        <h1>test</h1>
        <Select style={{ width: 120 }} onChange={(e)=>this.handleChange(e)}>
          {options}
        </Select>
        {
          this.state.camera ?
            <Calibrator name={ this.state.camera } /> :
            <p>No camera selected</p>
        }
      </div>
    );
  }
}

export default Index;
