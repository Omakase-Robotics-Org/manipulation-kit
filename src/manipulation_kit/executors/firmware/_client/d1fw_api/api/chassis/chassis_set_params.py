from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_param_type_0 import ChassisParamType0
from ...models.chassis_param_type_1 import ChassisParamType1
from ...models.chassis_param_type_2 import ChassisParamType2
from ...models.chassis_param_type_3 import ChassisParamType3
from ...models.chassis_param_type_4 import ChassisParamType4
from ...models.chassis_param_type_5 import ChassisParamType5
from ...models.chassis_set_params_response_200 import ChassisSetParamsResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ChassisParamType0
    | ChassisParamType1
    | ChassisParamType2
    | ChassisParamType3
    | ChassisParamType4
    | ChassisParamType5,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/set_params",
    }

    if (
        isinstance(body, ChassisParamType0)
        or isinstance(body, ChassisParamType1)
        or isinstance(body, ChassisParamType2)
        or isinstance(body, ChassisParamType3)
        or isinstance(body, ChassisParamType4)
    ):
        _kwargs["json"] = body.to_dict()
    else:
        _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisSetParamsResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisSetParamsResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisSetParamsResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisParamType0
    | ChassisParamType1
    | ChassisParamType2
    | ChassisParamType3
    | ChassisParamType4
    | ChassisParamType5,
) -> Response[ChassisSetParamsResponse200 | ErrorEnvelope]:
    """Write one mobile-base parameter

     Writes one of the vendor's six mobile-base parameters: the maximum driving speed (`max`), the local
    planner's slow-down/strong-light and slope zone speed limits (`low_or_strong`, `slope`), the
    charging-dock stop distance (`dist_stop`) or the footprint polygon (`footprint`). The sixth,
    `narrow`, is refused: the vendor writes it to a different file from the one its own reader reports
    it from, so the write can never be read back. The reply carries the conditions that decide whether
    the write does anything: `effective` is false on a mobile-base firmware generation whose firmware
    ignores the endpoint, `vendor_file` names the chassis-PC YAML the value landed in, and
    `applies_when` says which restart the base needs before it uses the value, because no running node
    re-reads those files. Gated by the chassis soft-kill latch, because every parameter changes how the
    base will move.

    Args:
        body (ChassisParamType0 | ChassisParamType1 | ChassisParamType2 | ChassisParamType3 |
            ChassisParamType4 | ChassisParamType5): One chassis parameter write, as `POST
            /v1/chassis/set_params` and the
            WebSocket `chassis.set_params` method take it.

            The vendor's `setParams` endpoint writes one of SIX settings on the mobile
            base, each into its own YAML file on the chassis PC. This is the typed
            form of that request: `{"kind": "max", "value": 0.3}`,
            `{"kind": "low_or_strong", "value": 0.4}`, `{"kind": "slope", "value":
            0.5}`, `{"kind": "narrow", "value": 0.4}`, `{"kind": "dist_stop",
            "value": 0.3}`, or `{"kind": "footprint", "value": [[x, y], [x, y],
            [x, y], [x, y]]}`.

            [`ChassisParam::Narrow`] is accepted by this type and REFUSED by the
            daemon; see its own documentation for why.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisSetParamsResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisParamType0
    | ChassisParamType1
    | ChassisParamType2
    | ChassisParamType3
    | ChassisParamType4
    | ChassisParamType5,
) -> ChassisSetParamsResponse200 | ErrorEnvelope | None:
    """Write one mobile-base parameter

     Writes one of the vendor's six mobile-base parameters: the maximum driving speed (`max`), the local
    planner's slow-down/strong-light and slope zone speed limits (`low_or_strong`, `slope`), the
    charging-dock stop distance (`dist_stop`) or the footprint polygon (`footprint`). The sixth,
    `narrow`, is refused: the vendor writes it to a different file from the one its own reader reports
    it from, so the write can never be read back. The reply carries the conditions that decide whether
    the write does anything: `effective` is false on a mobile-base firmware generation whose firmware
    ignores the endpoint, `vendor_file` names the chassis-PC YAML the value landed in, and
    `applies_when` says which restart the base needs before it uses the value, because no running node
    re-reads those files. Gated by the chassis soft-kill latch, because every parameter changes how the
    base will move.

    Args:
        body (ChassisParamType0 | ChassisParamType1 | ChassisParamType2 | ChassisParamType3 |
            ChassisParamType4 | ChassisParamType5): One chassis parameter write, as `POST
            /v1/chassis/set_params` and the
            WebSocket `chassis.set_params` method take it.

            The vendor's `setParams` endpoint writes one of SIX settings on the mobile
            base, each into its own YAML file on the chassis PC. This is the typed
            form of that request: `{"kind": "max", "value": 0.3}`,
            `{"kind": "low_or_strong", "value": 0.4}`, `{"kind": "slope", "value":
            0.5}`, `{"kind": "narrow", "value": 0.4}`, `{"kind": "dist_stop",
            "value": 0.3}`, or `{"kind": "footprint", "value": [[x, y], [x, y],
            [x, y], [x, y]]}`.

            [`ChassisParam::Narrow`] is accepted by this type and REFUSED by the
            daemon; see its own documentation for why.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisSetParamsResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisParamType0
    | ChassisParamType1
    | ChassisParamType2
    | ChassisParamType3
    | ChassisParamType4
    | ChassisParamType5,
) -> Response[ChassisSetParamsResponse200 | ErrorEnvelope]:
    """Write one mobile-base parameter

     Writes one of the vendor's six mobile-base parameters: the maximum driving speed (`max`), the local
    planner's slow-down/strong-light and slope zone speed limits (`low_or_strong`, `slope`), the
    charging-dock stop distance (`dist_stop`) or the footprint polygon (`footprint`). The sixth,
    `narrow`, is refused: the vendor writes it to a different file from the one its own reader reports
    it from, so the write can never be read back. The reply carries the conditions that decide whether
    the write does anything: `effective` is false on a mobile-base firmware generation whose firmware
    ignores the endpoint, `vendor_file` names the chassis-PC YAML the value landed in, and
    `applies_when` says which restart the base needs before it uses the value, because no running node
    re-reads those files. Gated by the chassis soft-kill latch, because every parameter changes how the
    base will move.

    Args:
        body (ChassisParamType0 | ChassisParamType1 | ChassisParamType2 | ChassisParamType3 |
            ChassisParamType4 | ChassisParamType5): One chassis parameter write, as `POST
            /v1/chassis/set_params` and the
            WebSocket `chassis.set_params` method take it.

            The vendor's `setParams` endpoint writes one of SIX settings on the mobile
            base, each into its own YAML file on the chassis PC. This is the typed
            form of that request: `{"kind": "max", "value": 0.3}`,
            `{"kind": "low_or_strong", "value": 0.4}`, `{"kind": "slope", "value":
            0.5}`, `{"kind": "narrow", "value": 0.4}`, `{"kind": "dist_stop",
            "value": 0.3}`, or `{"kind": "footprint", "value": [[x, y], [x, y],
            [x, y], [x, y]]}`.

            [`ChassisParam::Narrow`] is accepted by this type and REFUSED by the
            daemon; see its own documentation for why.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisSetParamsResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisParamType0
    | ChassisParamType1
    | ChassisParamType2
    | ChassisParamType3
    | ChassisParamType4
    | ChassisParamType5,
) -> ChassisSetParamsResponse200 | ErrorEnvelope | None:
    """Write one mobile-base parameter

     Writes one of the vendor's six mobile-base parameters: the maximum driving speed (`max`), the local
    planner's slow-down/strong-light and slope zone speed limits (`low_or_strong`, `slope`), the
    charging-dock stop distance (`dist_stop`) or the footprint polygon (`footprint`). The sixth,
    `narrow`, is refused: the vendor writes it to a different file from the one its own reader reports
    it from, so the write can never be read back. The reply carries the conditions that decide whether
    the write does anything: `effective` is false on a mobile-base firmware generation whose firmware
    ignores the endpoint, `vendor_file` names the chassis-PC YAML the value landed in, and
    `applies_when` says which restart the base needs before it uses the value, because no running node
    re-reads those files. Gated by the chassis soft-kill latch, because every parameter changes how the
    base will move.

    Args:
        body (ChassisParamType0 | ChassisParamType1 | ChassisParamType2 | ChassisParamType3 |
            ChassisParamType4 | ChassisParamType5): One chassis parameter write, as `POST
            /v1/chassis/set_params` and the
            WebSocket `chassis.set_params` method take it.

            The vendor's `setParams` endpoint writes one of SIX settings on the mobile
            base, each into its own YAML file on the chassis PC. This is the typed
            form of that request: `{"kind": "max", "value": 0.3}`,
            `{"kind": "low_or_strong", "value": 0.4}`, `{"kind": "slope", "value":
            0.5}`, `{"kind": "narrow", "value": 0.4}`, `{"kind": "dist_stop",
            "value": 0.3}`, or `{"kind": "footprint", "value": [[x, y], [x, y],
            [x, y], [x, y]]}`.

            [`ChassisParam::Narrow`] is accepted by this type and REFUSED by the
            daemon; see its own documentation for why.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisSetParamsResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
